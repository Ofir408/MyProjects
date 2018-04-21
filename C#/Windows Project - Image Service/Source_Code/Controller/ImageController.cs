using ImageService.Commands;
//using ImageService.Infrastructure;
//using ImageService.Infrastructure.Enums;
using ImageService.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ImageService.Controller {
    public class ImageController : IImageController {
        // The Model Object
        private IImageServiceModel m_model;   
        // mapped between numbers to Commands.
        private Dictionary<int, ICommand> commands;

        public ImageController(IImageServiceModel model) {
            // Storing the Model Of The System
            
            m_model = model;
            commands = new Dictionary<int, ICommand> {
                // For Now will contain just NEW_FILE_COMMAND
                { 1, new NewFileCommand(m_model) }
            }; 
        }
        
        
        public string ExecuteCommand(int commandID, string[] args, out bool resultSuccesful) {
            ICommand selectedCommand = commands[commandID];
            string returnedStr = selectedCommand.Execute(args, out resultSuccesful);
            return returnedStr;
        }
    }
}
