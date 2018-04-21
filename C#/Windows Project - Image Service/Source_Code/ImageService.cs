using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.ServiceProcess;
using System.Text;
using System.Runtime.InteropServices;
using ImageService.Model;
using ImageService.Controller;
using ImageService.Handler;
using ImageService.Server;
using ImageService.Logging;

namespace ImageService {
    // code to implement service pending status
    public enum ServiceState
    {
        SERVICE_STOPPED = 0x00000001,
        SERVICE_START_PENDING = 0x00000002,
        SERVICE_STOP_PENDING = 0x00000003,
        SERVICE_RUNNING = 0x00000004,
        SERVICE_CONTINUE_PENDING = 0x00000005,
        SERVICE_PAUSE_PENDING = 0x00000006,
        SERVICE_PAUSED = 0x00000007,
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct ServiceStatus
    {
        public int dwServiceType;
        public ServiceState dwCurrentState;
        public int dwControlsAccepted;
        public int dwWin32ExitCode;
        public int dwServiceSpecificExitCode;
        public int dwCheckPoint;
        public int dwWaitHint;
    };
    // end code to implement service pending status

    public partial class ImageService : ServiceBase
    {
        private ImageServer imgServer;
        private ILog logger;

        private System.ComponentModel.IContainer components;
        private System.Diagnostics.EventLog eventLog;
        private ServiceStatus serviceStatus;

        // below line is used to implement start pending status for the service
        [DllImport("advapi32.dll", SetLastError = true)]
        private static extern bool SetServiceStatus(IntPtr handle, ref ServiceStatus serviceStatus);

        //  identifier of the event to write into the event log. (starts at 1)
        private int eventId = 1;      

        public ImageService() {
            InitializeComponent();

            // fetch source and log name from app configuration
            eventLog.Source = AppConfig.Instance.SourceName;
            eventLog.Log = AppConfig.Instance.LogName;
            // if log doesnt exist, create it
            if (!System.Diagnostics.EventLog.SourceExists(eventLog.Source))
                System.Diagnostics.EventLog.CreateEventSource(eventLog.Source, eventLog.Log);


            // creates logger, imagecontroller, imageModel and ImageServer:
            logger = new LoggingModel();
            logger.MessageRecieved += LogEntry; //assign LogEntry to the logger event
            IImageController imgController = new ImageController(new ImageServiceModel(
            AppConfig.Instance.OutputDir, AppConfig.Instance.ThumbnailSize));
            List<string> paths = AppConfig.Instance.ListeningFolders;
            imgServer = new ImageServer(paths, imgController, logger);
        }

        // method to log an event in the service log
        private void LogEntry(object sender, LogArgs args) {
            EventLogEntryType msgType;
            switch (args.MsgType) {
                case Logging.Type.INFO:
                    msgType = EventLogEntryType.Information;
                    break;
                case Logging.Type.ERROR:
                    msgType = EventLogEntryType.Error;
                    break;
                case Logging.Type.WARNING:
                    msgType = EventLogEntryType.Warning;
                    break;
                default:
                    msgType = EventLogEntryType.Information;
                    break;
            }
            eventLog.WriteEntry(args.Msg, msgType, eventId++);
        }
        
        protected override void OnStart(string[] args) {
            // Update the service state to Start Pending.  
            serviceStatus = new ServiceStatus();
            serviceStatus.dwCurrentState = ServiceState.SERVICE_START_PENDING;
            serviceStatus.dwWaitHint = 100000;
            SetServiceStatus(ServiceHandle, ref serviceStatus);
                        
            // Update the service state to Running. (keep at end of OnStart)  
            serviceStatus.dwCurrentState = ServiceState.SERVICE_RUNNING;
            SetServiceStatus(ServiceHandle, ref serviceStatus);
            logger.Log("ImageService started");
        }

        protected override void OnStop() {
            imgServer.Close();
            logger.Log("ImageService stopped");
        }

        private void EventLog_EntryWritten(object sender, EntryWrittenEventArgs e) {

        }

        // used to be able to run the service in a console
        public void RunAsConsole(string[] args) {
            OnStart(args);
            Console.WriteLine("Press any key to exit...");
            Console.ReadLine();
            OnStop();
        }

    }
}
