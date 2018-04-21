using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ImageService.Logging {
    class LoggingModel : ILog {
        // the event that will be invoked
        public event EventHandler<LogArgs> MessageRecieved;
        // the method to call in order to log something
        public void Log(string msg, Type type = Type.INFO) {
            MessageRecieved?.Invoke(this, new LogArgs(msg,type));
        }
    }
}
