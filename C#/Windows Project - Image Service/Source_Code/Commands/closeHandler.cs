using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ImageService.Commands {
    class CloseHandler : ICommand {
        public string Execute(string[] args, out bool result) {
            //implement later when more commands will be added to server->handler
            result = true;
            return null;
        }
    }
}