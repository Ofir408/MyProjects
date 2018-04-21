using ImageService.Controller;
using ImageService.Handler;
using ImageService.Commands;
using System;
using System.Collections.Generic;
using ImageService.Logging;
using System.IO;


namespace ImageService.Server {
    public class ImageServer {
        #region Members
        private List<string> pathsToListen;
        private IImageController m_controller;
        private List<IHandler> handlers;
        private ILog m_logger;
        #endregion

        public ImageServer(List<string> paths, IImageController controller, ILog logger) {
            pathsToListen = paths;
            m_controller = controller;
            handlers = new List<IHandler>();
            m_logger = logger;
            
            BuildHandlers();
        }

        // creates a new handler for each folder
        public void BuildHandlers() {
            foreach (string s in pathsToListen) {
                if (Directory.Exists(s)) {
                    IHandler h = new FolderHandler(s, ref m_controller, ref m_logger);
                    h.HandlerClosing += HandlerClosed;
                    handlers.Add(h);
                    h.Begin();
                }
            }
        }

        public void Close() {
            // close all the handlers of the server.
            foreach (IHandler handler in handlers) {
                handler.ExecuteCommand(new CloseHandler());
            }
        }

        // method that will be called when one of the handlers is closing (its closing event is raised).
        private void HandlerClosed(object sender, EventArgs args) {
            // find the handler that is being closed
            IHandler handlerToRemove = handlers.Find(x => x == sender);
            // create a temporary list (can't update original list in case of running 'foreach' command)
            List<IHandler> temp = new List<IHandler>(handlers);
            temp.Remove(handlerToRemove);
            handlers = temp;
        }
    }
}