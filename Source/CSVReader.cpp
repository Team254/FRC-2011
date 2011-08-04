#include "CSVReader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <assert.h>

//TODO(ebakan): clean up dmitriy's hackish code
template <class T>
bool from_string(T& t, 
		const std::string& s, 
		std::ios_base& (*f)(std::ios_base&))
{
	std::istringstream iss(s);
	return !(iss >> f >> t).fail();
}

static std::string ConvertToLower(std::string const& inString)
{
	std::string outString;
	outString.reserve(inString.capacity());
	for (uint32_t i=0; i<inString.length(); i++) {
		outString+=tolower(inString[i]);
	}
	return outString;
}

CSVReader::CSVReader(const std::string& filePath)
: m_filePath(filePath), m_map()
{
	m_lock=semMCreate(SEM_Q_PRIORITY |
					 SEM_DELETE_SAFE |
					 SEM_INVERSION_SAFE);
	FillMap();
}

CSVReader::~CSVReader()
{
	semTake(m_lock,WAIT_FOREVER);
	semDelete(m_lock);
}

void CSVReader::ReloadValues()
{
	semTake(m_lock,WAIT_FOREVER);
	m_map.clear();
	FillMap();
	semGive(m_lock);
}

bool CSVReader::KeyExists(const std::string& key) const
{
	return (m_map.count(ConvertToLower(key)) > 0);
}

double CSVReader::GetValue(const std::string& valueName)
{
	return GetValueWithDefault(valueName, 0.0);
}
double CSVReader::GetValueWithDefault(const std::string& valueName, double def)
{
	double ans;
	semTake(m_lock,WAIT_FOREVER);
	if (KeyExists(valueName)) {
		ans = (*m_map.find(ConvertToLower(valueName))).second;
	} else {
		printf("USING DEFAULT OF %f FOR %s\n", def, valueName.c_str());
		ans = def;
	}
	semGive(m_lock);
	return ans;
}

void CSVReader::FillMap()
{
	std::string key;
	std::string valueString;
	
	std::ifstream infile (m_filePath.c_str());
	
	if (infile.is_open()) {	
		while (!infile.eof()) {
			std::string inKey;
			//Get the key name
			getline (infile, inKey, ',');
			key = ConvertToLower(inKey);
			
			// get the value string
			getline(infile, valueString);
			
			// Handle weird new line situations
			if(infile.eof())
				break;
			
			// Convert value string to a number
			double value = 0.0;
			if (from_string<double>(value, valueString, std::dec)) {
				m_map[key] = value;
				printf("RobotConfig:: Reading in Data Key %s Number %.2f %f\n", key.c_str(), value, (*m_map.find(key)).second);
			} else {
				//ensure we're always reading the right values
				assert(0);
				fprintf(stderr, "ERROR: Reading Key %s of Value %s failed\n", key.c_str(), valueString.c_str());
			}
		}
		infile.close();
	} else {
		fprintf(stderr, "ERROR: Unable to open config file %s \n", m_filePath.c_str()); 	
	}
}
