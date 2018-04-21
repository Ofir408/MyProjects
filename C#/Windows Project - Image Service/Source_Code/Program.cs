using System;
using System.Collections.Generic;
using System.Linq;
using System.ServiceProcess;
using System.Text;

namespace ImageService
{
    static class Program
    {
        static void Main(string[] args)
        {
            // if service is starting in console
            if (Environment.UserInteractive) {
                Console.WriteLine("Running service in console mode - for testing purposes only");
                Console.WriteLine("Please install the service using installutil.exe\n");
                ImageService service = new ImageService();
                service.RunAsConsole(args);
            }
            else {
                ServiceBase[] ServicesToRun;
                ServicesToRun = new ServiceBase[]
                {
                new ImageService()
                };
                ServiceBase.Run(ServicesToRun);
            }
        }
    }
}
