using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Text.RegularExpressions;
using System.Windows.Media.Imaging;
using ImageService.Controller;
using ImageService.Commands;
using System.Threading;
using System.Threading.Tasks;
using ImageService.Logging;

namespace ImageService.Handler {
    class FolderHandler : IHandler {
        private IImageController m_controller;       
        private ILog m_logger;
        private string m_folderPath;
        // file types to look for
        private static readonly string[] filters = { "*.jpg", "*.bmp", "*.png", "*.gif" };
        // list of watchers for each filter type
        List<FileSystemWatcher> watchers = new List<FileSystemWatcher>();
        public event EventHandler HandlerClosing;

        public FolderHandler(string folderPath, ref IImageController controller, ref ILog logger) {
            m_folderPath = folderPath;
            m_controller = controller;
            m_logger = logger;
        }

        public void Begin() {
            // add watcher for each file type
            foreach (string f in filters) {
                FileSystemWatcher w = new FileSystemWatcher();
                w.Path = m_folderPath;
                w.Filter = f;
                w.Created += FileCreated;
                w.EnableRaisingEvents = true;
                watchers.Add(w);
            }
        }
        // stop listening to a folder & dispose of the watchers
        private void Stop() {
            foreach (FileSystemWatcher watcher in watchers) {
                watcher.EnableRaisingEvents = false;
                watcher.Dispose();
            }
            HandlerClosing?.Invoke(this, null);
            m_logger.Log($"Stopped handling folder {m_folderPath}");
        }
        // method that will be raised when a new file is created in the folder
        private void FileCreated(object source, FileSystemEventArgs fileArgs) {
            Task t = new Task(() => {
                string sourceFilePath = fileArgs.FullPath;
                // wait for the new created file to be accessible (finish being created)
                while (!IsFileAccessible(sourceFilePath))
                    Thread.Sleep(500);

                string dest;
                // extract dateCreated from image
                if (GetImageTakenDate(sourceFilePath, out string yearCreated, out string monthCreated))
                    dest = GetOutputPath(yearCreated, monthCreated);
                else
                    dest = GetOutputPathNoDate();

                // execute addFile command (create thumbnail and copy image to new folder)
                string[] args = { sourceFilePath, dest };
                string errMsg = m_controller.ExecuteCommand(1, args, out bool success);
                if (success)
                    m_logger.Log($"successfuly handled {sourceFilePath} From {monthCreated}/{yearCreated}");
                else 
                    m_logger.Log($"Couldn't handle {sourceFilePath}, Reason: {errMsg}", Logging.Type.ERROR);
            });
            t.Start();
        }

        // extracts the date image taken from a given image path
        private bool GetImageTakenDate(string imgPath, out string year, out string month) {
            year = "";
            month = "";
            try {
                var fs = new FileStream(imgPath, FileMode.Open, FileAccess.Read, FileShare.Read);
                var img = BitmapFrame.Create(fs);
                var metadata = img.Metadata as BitmapMetadata;
                DateTime datePicTaken = DateTime.Parse(metadata.DateTaken);
                year = datePicTaken.Year.ToString();
                month = datePicTaken.Month.ToString();
                Console.WriteLine($"Got Date from {imgPath}, year={year} month={month}");
            }
            catch (Exception e) {
                Console.WriteLine("Couldnt resolve image date. path={0}\n exception={1}", imgPath, e);
                return false;
            }
            return true;
        }

        private bool IsFileAccessible(String sFilename) {
            // If the file can be opened for exclusive access it means that the file
            // is no longer locked by another process.
            try {
                using (FileStream inputStream = File.Open(sFilename, FileMode.Open, FileAccess.Read, FileShare.None)) {
                    if (inputStream.Length > 0)
                        return true;
                    else
                        return false;
                }
            }
            catch (Exception) {
                return false;
            }
        }

        private string GetOutputPath(string yearCreated, string monthCreated) {
            const string sep = @"\";
            return AppConfig.Instance.OutputDir + sep + yearCreated + sep + monthCreated;
        }
        private string GetOutputPathNoDate() {
            const string sep = @"\";
            return AppConfig.Instance.OutputDir + sep + "Undated" + sep + "Undated";
        }

        // handles only a single command atm
        public void ExecuteCommand(ICommand cmnd) {
            m_logger.Log($"Handler recieved command from server.");
            if (cmnd is CloseHandler)
                Stop();
        }
    }
}
