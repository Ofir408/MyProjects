using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ImageService.Logging {
    // log message types
    public enum Type {
        INFO,
        ERROR,
        WARNING
    }
    public class LogArgs : EventArgs {
        public LogArgs(string msg, Type t = Type.INFO) {
            MsgType = t;
            Msg = msg;
        }
        public Type MsgType { get; set; }
        public string Msg { get; set; }
    }
}
