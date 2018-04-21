using ImageService.Commands;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ImageService.Handler {
    public interface IHandler {
        void Begin(); //start handling the folder
        void ExecuteCommand(ICommand cmnd);
        event EventHandler HandlerClosing;
    }
}