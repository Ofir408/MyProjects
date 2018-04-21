using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Configuration;
using System.IO;

namespace ImageService {
    //singleton class used to fetch app configuration (read-only atm)
    public class AppConfig {
        private string handler, outputDir, sourceName, logName;
        int thumbnailSize;
        private List<string> listeningFolders;

        public static AppConfig Instance {
            get {
                if (instance == null)
                    instance = new AppConfig();
                return instance;
            }
        }

        public string OutputDir {
            get { return outputDir; }
        }
        public string SourceName {
            get { return sourceName; }
        }
        public string LogName {
            get { return logName; }
        }
        public int ThumbnailSize {
            get { return thumbnailSize; }
        }
        public List<string> ListeningFolders {
            get { return listeningFolders; }
        }

        private static AppConfig instance;
        //constructor
        private AppConfig() {
            ReadAllSettings();
            listeningFolders = ExtractListeningFolders();
        }

        private void ReadAllSettings() {
            // defaults
            const string defHandler = "";
            const string defSourceName = "ImageServiceSource";
            const string defLogName = "ImageServiceLog";
            const int defThumbnailSize = 120;
            string defOutputDir = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + @"\images";

            // try reading each setting, if unsuccessful use default
            if (!ReadSetting("OutputDir", out outputDir))
                outputDir = defOutputDir;
            if (!ReadSetting("Handler", out handler))
                handler = defHandler;
            if (!ReadSetting("SourceName", out sourceName))
                sourceName = defSourceName;
            if (!ReadSetting("LogName", out logName))
                logName = defLogName;
            if (!ReadSetting("ThumbnailSize", out string strThumbnailSize) ||
                !Int32.TryParse(strThumbnailSize, out thumbnailSize))
                thumbnailSize = defThumbnailSize;
        }

        // read a single setting from the configuration manager, result will be stored to val
        // method return true if successful and false if key non existing/errored
        private bool ReadSetting(string key, out string val) {
            try {
                val = ConfigurationManager.AppSettings[key];
                if (val == null)
                    return false;
                return true;
            }
            catch (ConfigurationErrorsException e) {
                val = null;
                Console.WriteLine($"Error reading app setting {key}, details: {e}\n");
                return false;
            }
        }

        // helper method the gets a string : handlerPath, that has a list of
        // paths of folders that we need to listen to them, each seperated by ;
        // will put each path as an item in list
        // For example: C:\Users\Downloads; C:\Users\Pictures, will split to:
        // list in place [0] = "C:\Users\Downloads", place[1] = C:\Users\Pictures.
        private List<string> ExtractListeningFolders() {
            const char seperator = ';';
            string[] listToReturn = handler.Split(seperator);
            List<string> temp = listToReturn.ToList();
            temp.RemoveAt(temp.Count - 1);
            return temp; // remove last element to avoid from path "".

        }
    }
}
