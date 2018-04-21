using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ImageService.Model
{
    public interface IImageServiceModel
    {
        /// <summary>
        /// The Function Addes A file to the destPath
        /// Add the image and opening the needed folder according year & month.
        /// Moreover, create the small picture and put it in the needed folders, if they not exist,
        /// create them before.
        /// </summary>
        /// <param name="path">The Path of the Image from the file</param>
        /// <returns>Indication if the Addition Was Successful</returns>
        // returns a string - "Successful" if does his task as expected, 
        //                  Otherwise a string that explains about the problem the accured.
        // Moreover, (bool) result is true if the function succeeded, otherwise false.
        string AddFile(string sourcePath, string destPath, out bool result);

        // Function that deletes a file.
        // Gets: path -> the path of the file that need to be removed.
        // returns a string - "Successful" if does his task as expected, 
        //                  Otherwise a string that explains about the problem the accured.
        // Moreover, (bool) result is true if the function succeeded, otherwise false.
        string DeleteFile(string path, out bool result);

        // Function that moves a file from source path to destPath.
        // returns a string - "Successful" if does his task as expected, 
        //                  Otherwise a string that explains about the problem the accured.
        // Moreover, (bool) result is true if the function succeeded, otherwise false. 
        string MoveFile(string soucrePath, string destPath, out bool result);

        // For example if we want to create new path into library b into a that called myLib, 
        // So we need to call this function in the that way:
        // CreateFolder((sourcePath), myLib, booleanVar); 
        // This method create nameOfNewFolder library into the input path (sourcePath).
        string CreateFolder(string sourcePath, string nameOfNewFolder, out bool result);

    }
}
