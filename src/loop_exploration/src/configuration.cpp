/**
 * This file is part of LAEG
 *
 * Copyright 2022 Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * For more information see <https://github.com/phir2-lab/laeg>
 *
 * LAEG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LAEG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LAEG If not, see <https://www.gnu.org/licenses/>.
**/

#include "configuration.h"

Configuration::Configuration(){}

Configuration::Configuration(string adress)
{
    this->Load(adress);
}

bool Configuration::Load(string adress)
{
    ifstream file;
    file.open(adress);

    if(!file.is_open())
    {
        cout << endl << "[ERROR] Cannot read configuration file: " << adress << endl;
        return false;
    }

    string line;
    while(getline(file,line))
    {
        line = RemoveComment(line);
        istringstream is_line(line);
        string key;
        if(getline(is_line, key, '='))
        {
            string value;
            if(getline(is_line, value) )
            {
                key = RemoveSpace(key);
                value = RemoveSpace(value);
                values_[key] = value;
            }

        }

    }

    return true;
}

bool Configuration::GetString(string key, string& dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = it->second;
        return true;
    }

    return false;
}

bool Configuration::GetInt(string key, int& dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stoi(it->second);
        return true;
    }

    return false;
}

bool Configuration::GetFloat(string key, float &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stof(it->second);
        return true;
    }

    return false;
}

bool Configuration::GetDouble(string key, double &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        dst = stod(it->second);
        return true;
    }

    return false;
}

bool Configuration::GetBool(string key, bool &dst)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        if(it->second.compare("true") == 0 || it->second.compare("True") == 0 || it->second.compare("TRUE") == 0)
        {
            dst = true;
            return true;
        }
        else if(it->second.compare("false") == 0 || it->second.compare("False") == 0 || it->second.compare("FALSE") == 0)
        {
            dst = false;
            return true;
        }
        else{
            cout << endl << "[WARNING] Parameter " << key << " in configurations is not a correct bool expression. (" << it->second << ")" << endl;
            return false;
        }
    }

    return false;
}

string Configuration::GetString(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
        return it->second;

    cout << endl << "[WARNING] Cannot find parameter " << key << " in configurations." << endl;
    return "";
}

int Configuration::GetInt(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
        return stoi(it->second);

    cout << endl << "[WARNING] Cannot find parameter " << key << " in configurations." << endl;
    return -1;
}

float Configuration::GetFloat(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        istringstream buffer(it->second);
        float number;
        buffer >> number;
        return number;
    }

    cout << endl << "[WARNING] Cannot find parameter " << key << " in configurations." << endl;
    return -1;
}

double Configuration::GetDouble(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        istringstream buffer(it->second);
        double number;
        buffer >> number;
        return number;
    }

    cout << endl << "[WARNING] Cannot find parameter " << key << " in configurations." << endl;
    return -1;
}

bool Configuration::GetBool(string key)
{
    map<string, string>::iterator it = values_.find(key);

    if (it != values_.end())
    {
        if(it->second.compare("true") == 0 || it->second.compare("True") == 0 || it->second.compare("TRUE") == 0)
        {
            return true;
        }
        else if(it->second.compare("false") == 0 || it->second.compare("False") == 0 || it->second.compare("FALSE") == 0)
        {
            return false;
        }
        else{
            cout << endl << "[WARNING] Parameter " << key << " in configurations is not a correct bool expression.(" << it->second << ")" << endl;
            return false;
        }
    }

    cout << endl << "[WARNING] Cannot find parameter " << key << " in configurations." << endl;
    return false;
}

const string Configuration::RemoveSpace(string original)
{
    for (size_t i = 0; i < original.length(); i++)
    {
        if(original[i] == ' ' || original[i] == '\n' || original[i] == '\t') {
            original.erase(i, 1);
            i--;
        }
    }
    return original;
}

const string Configuration::RemoveComment(string original)
{
    string comment_marker = "#";
    size_t pos = 0;
    string token;

    while ((pos = original.find(comment_marker)) != string::npos) {
        token = original.substr(0, pos);
        original.erase(pos, string::npos);
    }
    return original;
}
