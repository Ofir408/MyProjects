// using ImageService.Infrastructure;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace ImageService.Model {
    public class ImageServiceModel : IImageServiceModel {

        // when new images are added the service will arange the pictures in OutputDir library,
        // according to year and a month (when the picture was created). 
        // Moreover, for each pictures that adds to OutputDir,
        // the service will create Thumbnail with the size : m_thumbnailSize 


        #region Members
        private string m_OutputFolder;            // The Output Folder
        private int m_thumbnailSize;              // The Size Of The Thumbnail Size
        #endregion

        public ImageServiceModel(string outputFolder, int sizeOfThumbnail) {
            m_OutputFolder = outputFolder;
            m_thumbnailSize = sizeOfThumbnail;
        }


        public string MoveFile(string sourcePath, string destPath, out bool result) {
            try {
                if(System.IO.Directory.Exists(destPath)) {
                    string d = destPath + @"\" + Path.GetFileName(sourcePath);
                    System.IO.File.Copy(sourcePath, d, true);
                    DeleteFile(sourcePath, out result);
                    if(result == true)
                        return "Successful";
                }
                // otherwise: 
                result = false; // can't move the file to dest.
                return "Into MoveFile, can't find target path";

            } catch(Exception e) {
                result = false;
                Console.WriteLine(e.ToString());
                return e.ToString();
            }
        }

        // gets direct path to remove. 
        // For example if path is: ../m_OutputFolder/photoOne.png
        // This function delets photoOne.png from ../m_OutputFolder
        public string DeleteFile(string path, out bool result) {

            try {
                if(File.Exists(path)) {
                    File.Delete(path);
                }

                result = true;
                return "Successful";
            } catch(Exception e) {
                // failed in the task because there was a problem.
                result = false;
                return e.ToString();
            }
        }

        // helper function that helps us to interpret the folder of the year 
        // and the folder of the month, from the input of the full path string 
        // that we got.
        // For example if we get ../m_OutputFolder/1998/05 so we will put year = 1998, month = 05.
        public void DataInterpreter(string inputStr, out string year, out string month) {
            try {
                int startingIndex = inputStr.IndexOf(m_OutputFolder) + m_OutputFolder.Length;
                string tempSubStr = inputStr.Substring(startingIndex + 1);
                year = tempSubStr.Substring(0, tempSubStr.IndexOf(@"\"));
                string afterGettingYear = inputStr.Substring(startingIndex + year.Length + 1);
                string temp = afterGettingYear.Substring(1);
                try {
                    month = temp.Substring(0, temp.IndexOf(@"\"));
                } catch(ArgumentException) {
                    month = temp.Substring(0);
                }
            } catch(Exception) {
                year = "Undated";
                month = "Undated";
                Console.WriteLine("Failed to parse into ImageServiceModel:dataInterpreter\n data" +
                    " into year & month doesnt rellevant");
            }
        }
        // for example if sourcePath is: ../firstFolder And nameOfNewFolder is: newFolder
        // So we will create into firstFolder a new folder that is called newFolder.
        public string CreateFolder(string sourcePath, string nameOfNewFolder, out bool result) {
            try {
                string newFolderPath = System.IO.Path.Combine(sourcePath, nameOfNewFolder);
                if(Directory.Exists(newFolderPath)) {
                    // it means that no need to create because we allready have this folder.
                    result = true; // since we have the needed folder.
                    return "The folder allready Exists";
                }

                System.IO.Directory.CreateDirectory(newFolderPath);
                result = true;
                return "Successful";

            } catch(Exception e) {
                result = false;
                return e.ToString();
            }
        }

        // using OutputDirCreater to avoid from duplicate code.
        // For example, if we get sourcePath = "C:/Users/Ofir/Pictures/ex1/check.png"
        // And destPath = "C:/Users/Ofir/Pictures/ex1/Thumbnails/1995/08"
        // So we will put the small image into the dest folder after opening the needed folders.

        private string ThumbnailsCreater(string sourcePath, string destPath, out bool result) {
            Directory.CreateDirectory(destPath);

            Image image = Image.FromFile(sourcePath);
            Image thumb = image.GetThumbnailImage(m_thumbnailSize, m_thumbnailSize,
                () => false, IntPtr.Zero);
            // saving the Thumbnail image into the needed directory.
            string t = destPath + @"\" + Path.GetFileName(sourcePath);
            if(File.Exists(t)) {
                // we allready have a file in that name, therefore create it with diffrent name.
                Random rnd = new Random();
                int randNum = rnd.Next(1, 100000); // creates a number between 1 and 1000
                string temp = Convert.ToString(randNum);
                t = destPath + @"\" + temp + Path.GetFileName(sourcePath);
            }
            thumb.Save(t);

            result = true;
            return "Successful";
        }

        // For example, if we get sourcePath = "C:/Users/Ofir/Pictures/ex1/check.png"
        // And destPath = "C:/Users/Ofir/Pictures/ex1/1995/08"
        // So we will put the small image into the dest folder after opening the needed folders.
        private string OutputDirCreater(string sourcePath, string destPath, out bool result) {
            try {
                string year, month, updateSrcPathWithYear;

                // create outputDir as hidden folder if doesn't exists.

                if(!Directory.Exists(m_OutputFolder)) {
                    DirectoryInfo di = Directory.CreateDirectory(m_OutputFolder);
                    di.Attributes = FileAttributes.Directory | FileAttributes.Hidden;
                }


                DataInterpreter(destPath, out year, out month);
                CreateFolder(m_OutputFolder, year, out result);

                if(result == true) {
                    updateSrcPathWithYear = m_OutputFolder + @"\" + year;
                    CreateFolder(updateSrcPathWithYear, month, out result);
                    if(result == true) {
                        // if we here, it means that we have the folders.
                        string t = destPath + @"\" + Path.GetFileName(sourcePath);
                        if(File.Exists(t)) {
                            // we allready have a file in that name, therefore create it with diffrent name.
                            Random rnd = new Random();
                            int randNum = rnd.Next(1, 1000000); // creates a number between 1 and 1000000
                            string temp = Convert.ToString(randNum);
                            t = destPath + @"\" + temp + Path.GetFileName(sourcePath);
                            if(File.Exists(t)) {
                                randNum = rnd.Next(1, 1000000); // creates a number between 1 and 1000000
                                temp = Convert.ToString(randNum);
                                t = destPath + @"\" + temp + Path.GetFileName(sourcePath);
                            }
                        }
                        System.IO.File.Copy(sourcePath, t, result);
                        if(result == false) {
                            return "Failed to copy the file";
                        }
                        result = true;
                        return "Successful";
                    }
                }
                // if we here it means result is false
                result = false;
                return "Failed Into AddFile()";

            } catch(Exception e) {
                result = false;
                return e.ToString();
            }
        }

        // add the file into path to destPath.
        // For example if we get: sourcePath = ../folderOne/im.png
        // And destPath is: ../OutputFolder/1994/05
        // So we will copy the im.png to /OutputFolder/1994/05 (and opening the folders)
        // Moreover, will create the small image into ../OutputFolder/Thumbnails/1994/05.

        public string AddFile(string sourcePath, string destPath, out bool result) {
            string returnedStr;
            try {
                returnedStr = OutputDirCreater(sourcePath, destPath, out result);
                if(result == false) {
                    return returnedStr;
                } else {
                    // copy to Thumbnail folder.
                    // want to add Thumbnails to the destPath, for example instead of: ../OutputDir/year/month
                    // we want now: ../OutputDir/Thumbnail/year/month.
                    int startingIndex = destPath.IndexOf(m_OutputFolder) + m_OutputFolder.Length;
                    string tempSubStr = destPath.Substring(startingIndex + 1);
                    string updateDstStr = destPath.Substring(0, startingIndex + 1) + @"Thumbnails\" + tempSubStr;
                    returnedStr = ThumbnailsCreater(sourcePath, updateDstStr, out result);
                    if(result == false) {
                        return returnedStr;
                    } else {
                        GC.Collect();
                        Thread.Sleep(500);
                        File.Delete(sourcePath);
                    }
                    // we want to return the destPath that we entered the image into, 
                    // since the function succeeded to add the file as needed.
                    return destPath; // and result is true..
                }

            } catch(Exception e) {
                result = false;
                return e.ToString();
            }
        }
    }
}
