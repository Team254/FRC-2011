//class to read from .csv file

#ifndef CSV_READER_H
#define CSV_READER_H

#include <map>
#include <string>
#include <semLib.h>

/**
 * Class to read csv (comma separated value) file.
 *
 * The CSV is used for easy modification of constants and other input
 * to the robot. It can be easily modified with any text editor as well
 * as editors for spreadsheets (e.g. Excel and OpenOffice Calc).
 *
 * The data inside the CSV in this instance should be stored as a map
 * (keys assigned to values). The keys in this instance are strings, while
 * the values are doubles.
 */
class CSVReader {
public:
	/**
	 * Constructs a CSVReader corresponding to a location for
	 * a CSV file.
	 *
	 * This method initializes the CSV by reading all key-value pairs in
	 * the CSV and storing them in a map. This is done once unless the
	 * <code>reloadValues()</code> function is called.
	 *
	 * @param filePath the CSV file to read
	 */
	CSVReader(const std::string& filePath);
	~CSVReader();

	/**
	 * Returns the value associated with the CSV string key.
	 *
	 * If the key does not exist in the table, returns the default value 0.0.
	 *
	 * @param valueName the key which to search for
	 * @return the double value assigned to the key, or 0.0 if the key is not
	 * found
	 */
	double GetValue(const std::string& valueName);

	/**
	 * Returns the value associated with the CSV string key, with
	 * a default value provided.
	 *
	 * In other words, if the key does not exist in the table,
	 * the default value is returned.
	 *
	 * @param valueName the key which to search for
	 * @param def the value to default to if the key does not exist
	 * @return the double value assigned to the key, or <code>def</code> if
	 * the key is not found.
	 */
	double GetValueWithDefault(const std::string& valueName, double def);

	/**
	 * Re-reads the CSV file, reloading the CSV values into the map.
	 */
	void ReloadValues();
private:
	bool KeyExists(const std::string& valueName) const;
	SEM_ID m_lock;
	void FillMap();
	std::string m_filePath;
	std::map<std::string, double> m_map;
	
};
#endif // CSV_READER_H
