/**
* @brief  Functions for reading/writing data into file
*
* @author Yiming Wang (wangyimingkaren@gmail.com)
* @date 16/09/2016
*/

#ifndef FILE_UTIL_HPP_INCLUDED
#define FILE_UTIL_HPP_INCLUDED

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <sys/stat.h>

using namespace std;

/**
 * @brief template function that converts input to string
 * @param input of type T, e.g. float, double, int ...
 */
template <typename T>
string to_string (T Number)
{
    ostringstream temp;
    temp<<Number;
    return temp.str();
}

namespace fileUtil
{

/**
* @brief empty a file
* @param file name in absolute path, note the input is in char* not in string type
*/
bool cleanfile(const char* filename)
{
    ofstream myfile;
    myfile.open (filename, ios::out | ios::trunc);
    bool retValue;

    if (myfile.is_open())
    {
        retValue =  true;
    }
    else
    {
        retValue = false;
    }

    myfile.close();
    return retValue;
}

/**
* @brief writes a string to a file in appending way
* @param file name in absolute path
* @param the string to write
*/
bool write2file(const char* filename, string string2write)
{

    ofstream myfile;
    myfile.open (filename, ios::out | ios::app);


    bool retValue;


    if (myfile.is_open())
    {
        // opened successfully now write to the file
        myfile << string2write;
        myfile.close();

        retValue = true;
    }
    else
    {
        cout << "Unable to open file...\n";
        retValue = true;
    }

    return retValue;
}

/**
* @brief reads the whole file and print it line by line
* @param file name in absolute path
*/
bool readWholefile(const char* filename)
{

    bool retVal;
    string line;

    ifstream myfile (filename);

    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            cout << line << '\n';
        }
        myfile.close();
        retVal = true;

    }
    else
    {
        cout << "Unable to open file";
        retVal = false;
    }

    return retVal;
}


/**
* @brief reads one line from a file and print it line by line
* @param the file indicator
* @return the string of next line
*/
string readOneLine(ifstream myfile)
{

    string line;
    //ifstream myfile (filename);

    if (myfile.is_open())
    {
        if (!getline (myfile,line))
        {
            cout<<"This is the end of the file, no line read...\n";
        }

        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
        line = "";
    }
    return line;
}

/**
* @brief check if the file exist
* @param file name in absolute path in string
* @return bool to tell if exist
*/
bool checkFileExistence(string filename)
{
    struct stat fileInfo;
    return stat(filename.c_str(), &fileInfo) == 0;
}

/**
* @brief check if the file name exist and always return a new name by appending version no.
* @param file name in absolute path in string without extension
* @param file type ext, e.g. '.txt' or '.avi'
* @return a new name
*/
string checkFileName(string filename, string extension)
{
    stringstream ss;
    string newFileName, temp_string;

    ss<<filename;

    struct stat fileInfo;
    bool ifContinune = true;
    int index_ = 1;

    while (ifContinune)
    {
        // sstream object is non-copiable, need _to give the content only
        temp_string = ss.str()+extension;
        bool existingFile = stat(temp_string.c_str(), &fileInfo) == 0;
        if (existingFile)
        {
            ss.str(""); // to clear the sstream
            ss<<filename<<"_"<<index_;
            index_++;
            ifContinune = true;
        }
        else
        {

            ss<<extension;
            ifContinune = false;
        }
    }

    return newFileName = ss.str();
}


}
#endif
