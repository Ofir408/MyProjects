using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ImageService.Logging {
    public interface ILog {
        // the event that will be invoked
        event EventHandler<LogArgs> MessageRecieved;
        // the method to call in order to log something
        void Log(string msg, Type type = Type.INFO);
    }
}