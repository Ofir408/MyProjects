//using ImageService.Infrastructure;
using ImageService.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ImageService.Commands {
    public class NewFileCommand : ICommand {

        // NewFileCommand takes care to add a new file.
        private IImageServiceModel m_model;


        public NewFileCommand(IImageServiceModel model) {
            m_model = model;            // Storing the Model
        }

        // covert to args to the data that we will send to m_model.AddFile(..)
        // The arguments that should be passed are: 
        // args[0] : sourcePath
        // args[1] : destPath
        public string Execute(string[] args, out bool result) {
            string returnedString, sourcePath, destPath;
            sourcePath = args[0];
            destPath = args[1];
            returnedString = m_model.AddFile(sourcePath, destPath, out result);
            // The String Will Return the New Path if result = true, and will return the error message
            if (result == true) {
                return destPath; 
            } else {
                return returnedString; // that has an error message.
            }
        }
    }
}
