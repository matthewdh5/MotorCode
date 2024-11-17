#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <boost/asio.hpp>

using namespace std;
using namespace boost::asio;
using boost::system::error_code;
using namespace std::chrono;

//CommandInfoTX is used for the universial command. Stores the information requred for this function to send any command to the motors. Refer to the motors DataSheet
// 
//Format is as such [ID][HexCMD][WholePackageSize][CMD_SIZE][#ofChecksums][Checksum1][Checksum2]...[ChecksumX][ResponseType][angle][speed][dir][torque]
// 
//All information is provided in the motor datasheet
//[ID]-This is the command number referenced in the datasheet ie (1)Read motor state 1 and error command,(2) Clear error command,... etc
//[HexCMD]-This is the actual command identifyer for the motors
//[[WholePackageSize]- size of the entire command
//[Data_SIZE]-The commands will have a header and actual data portion. If it has data beyond the header this is the size of it.
//[#ofChecksums]- Number of checksums in the the command
//[ChecksumX]- Indexed location of the checksums
//[ResponseType]-Type of response there is  only 2 :)
//[angle-torque]- 1 or 0 if we are using them in the command
int sizeof64 = 8;
int sizeof32 = 4;
int sizeof16 = 2;



vector<vector<uint8_t>>commandInfoTX = {
    {1,0x9A,5,0,1,4,1,0,0,0,0},
    {2,0x9B,5,0,1,4,1,0,0,0,0},
    {3,0x9C,5,0,1,4,2,0,0,0,0},
    {10,0xA1,8,2,2,4,7,2,0,0,0,1},
    {12,0xA3,14,8,2,4,13,2,1,0,0,0},
    {13,0xA4,18,10,2,4,17,2,1,1,0,0}
};
std::vector<std::vector<std::tuple<std::string, string, int, int>>> commandInfoRX = {
        {   //Data for Motor Data 2
            std::make_tuple("Temperature", "C", 5, 5),
            std::make_tuple("Torque", "N/m", 6, 7),
            std::make_tuple("Speed", "dps", 8, 9),
            std::make_tuple("Position", "Encoder", 10, 11)
        }
};
std::chrono::steady_clock::time_point start;
std::vector<double> elapsedTimes;

//Format for commandInfoRX {Name,Unit,Location StartLocation, EndLocation}
void writeVectorToCSV(const std::vector<double>& data, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing." << std::endl;
        return;
    }

    for (const auto& value : data) {
        file << value << std::endl; // Write each element on a new line
    }

    file.close();
}


void startTimer() {
    start = std::chrono::steady_clock::now();
}
void trackElapsedTime() {
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    elapsedTimes.push_back(elapsed.count());
}

void clearScreen() {
#ifdef _WIN32
    std::system("cls");
#else
    std::system("clear");
#endif
}


void printHexs(const vector<vector<vector<uint8_t>>>& csvData) {
    for (const auto& column : csvData) {
        for (const auto& row : column) {
            for (const auto& element : row) {
                cout << hex << setw(2) << setfill('0') << static_cast<int>(element) << " ";
            }
            cout << "  ";  // Add space between elements of the same row
        }
        cout << endl;  // Add a new line between rows
    }
}

void printIndices(const vector<vector<vector<uint8_t>>>& csvData, size_t columnIndex, size_t rowIndex, size_t elementIndex) {
    if (columnIndex >= csvData.size() || rowIndex >= csvData[0].size() || elementIndex >= csvData[0][0].size()) {
        cerr << "Invalid indices." << endl;
        return;
    }

    uint8_t value = csvData[columnIndex][rowIndex][elementIndex];

    cout << "Value at [" << columnIndex << "][" << rowIndex << "][" << elementIndex << "]: ";
    cout << hex << setw(2) << setfill('0') << static_cast<int>(value) << endl;
}

void printRowData(const vector<vector<vector<uint8_t>>>& csvData, size_t columnIndex, size_t rowIndex) {
    if (columnIndex >= csvData.size() || rowIndex >= csvData[columnIndex].size()) {
        cerr << "Invalid column or row index." << endl;
        return;
    }

    // Access and print each byte in the specified row of the specified column
    cout << "Data in column " << columnIndex << ", row " << rowIndex << ": ";
    for (const auto& byte : csvData[columnIndex][rowIndex]) {
        cout << hex << setw(2) << setfill('0') << static_cast<int>(byte) << " ";
    }
    cout << endl;
}

vector<vector<vector<uint8_t>>> readCSVFile(const string& csvFilePath) {
    ifstream file(csvFilePath, ios::binary);
    if (!file.is_open()) {
        cerr << "Error opening file: " << csvFilePath << endl;
        return {};
    }

    // Check for BOM and rewind if necessary
    unsigned char bom[3] = { 0xEF, 0xBB, 0xBF };
    char fileBom[3] = { 0 };
    file.read(fileBom, sizeof(fileBom));
    if (memcmp(bom, fileBom, sizeof(bom)) != 0) {
        // BOM not present, rewind to the beginning
        file.seekg(0);
    }

    vector<vector<vector<uint8_t>>> csvData;
    string line;

    while (getline(file, line)) {
        vector<vector<uint8_t>> columnData;
        istringstream ss(line);
        string token;
        while (getline(ss, token, '*')) {
            vector<uint8_t> rowData;
            istringstream tokenStream(token.substr(0, token.size() - 1));
            string hexValue;

            while (getline(tokenStream, hexValue, ',')) {
                if (!hexValue.empty()) {
                    try {
                        uint8_t value = static_cast<uint8_t>(stoi(hexValue, nullptr, 16));
                        rowData.push_back(value);
                    }
                    catch (const std::invalid_argument& e) {
                        // Handle the case where stoi encounters an invalid argument
                        cerr << "Error converting string to integer: " << e.what() << endl;
                        // Optionally, you can choose to skip this value or take another action
                    }
                }
            }
            columnData.push_back(rowData); // Using push_back to add rowData to columnData
        }

        csvData.push_back(columnData);
    }

    file.close();
    return csvData;
    /*
    // Transpose the data
    vector<vector<vector<uint8_t>>> transposedData;
    size_t numRows = csvData.size();
    size_t numCols = numRows > 0 ? csvData[0].size() : 0;

    for (size_t col = 0; col < numCols; ++col) {
        vector<vector<uint8_t>> transposedRow;
        for (size_t row = 0; row < numRows; ++row) {
            transposedRow.push_back(csvData[row][col]);
        }
        transposedData.push_back(transposedRow);
    }

    return transposedData;
    */
}

void multiloopControl(uint8_t motorID, int degreePosition, boost::asio::serial_port& serial) {
    int CMDSize = 14 - 1;
    uint8_t store = 0;
    //Size of entire command needs to be 14
    //Convert the degree position to a string of hex values
    //Checksum header  is [0]-[3]
    //Checksum command is [5]-[12]
   // motorID = 1;
    //degreePosition = 90;
    degreePosition = degreePosition * 1000;



    std::stringstream ss;
    ss << std::hex << degreePosition;
    string hexString = ss.str();
    if ((hexString.length() % 2 != 0)/* && hexString.length() != 16*/) {
        hexString = "0" + hexString;
    }

    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hexString.length(); i += 2) {
        std::string byteString = hexString.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoi(byteString, nullptr, 16));
        bytes.push_back(byte);
    }
    std::reverse(bytes.begin(), bytes.end());
    vector<uint8_t> command = {/*Header Data*/0x3E,0xA3,motorID,0x08,0x00,/*Command Data*/ };


    command.insert(command.end(), bytes.begin(), bytes.end());
    int size = command.size();
    //For loops insert Checsum
    while (command.size() <= CMDSize) {
        command.push_back(0x00);
    }
    for (int i = 0; i < 4; i++) {
        store += command[i];
    }
    command[4] = store;
    store = 0;
    for (int i = 5; i < CMDSize; i++) {
        store += command[i];
    }
    command[13] = store;

    write(serial, buffer(command.data(), command.size()));


}

void universialControl(int commandID, uint8_t motorID, int degreePosition, int dps, int torque, uint8_t dir, boost::asio::serial_port& serial) {

    /*DEBUG*/

    vector<int> checksumLoc;



    int location;
    int index = 0;
    //Finds the apprioate command data
    for (location = 0; commandInfoTX[location][index] != commandID; location++) {}

    //Stores hex value or the command
    uint8_t hexCMD = commandInfoTX[location][++index];
    //Stores size of the entire command
    int CMDSize = commandInfoTX[location][++index] - 1;
    //Size of the data portion of the command
    uint8_t dataSize = commandInfoTX[location][++index];
    index++;
    int numCheck = commandInfoTX[location][index];
    int i;
    for (i = 0; i < numCheck; i++) {
        checksumLoc.push_back(commandInfoTX[location][++index]);
    }
    int responseType = commandInfoTX[location][++index];
    int useAngle = commandInfoTX[location][++index];
    int useDps = commandInfoTX[location][++index];
    int useDir = commandInfoTX[location][++index];
    int useTorque = commandInfoTX[location][++index];

    degreePosition = degreePosition * 1000;
    dps = dps * 1000;
    vector<uint8_t> command = {/*Header Data ->*/0x3E,hexCMD,motorID,dataSize/*Command Data ->*/ };

    //Add first checksum
    uint8_t store = 0;
    for (i = 0; i < checksumLoc[0]; i++) {
        store += command[i];
    }

    command.push_back(store);

    //Inserts hex values for direction
    if (useDir == 1) {
        command.push_back(dir);

    }
    //Inserts hex values for direction


    if (useAngle == 1) {
        std::stringstream ss;
        ss << std::hex << degreePosition;
        string hexString = ss.str();
        if ((hexString.length() % 2 != 0)/* && hexString.length() != 16*/) {
            hexString = "0" + hexString;
        }
        //Converts string values into hex
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i < hexString.length(); i += 2) {
            std::string byteString = hexString.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(std::stoi(byteString, nullptr, 16));
            bytes.push_back(byte);
        }

        std::reverse(bytes.begin(), bytes.end());

        while (bytes.size() < sizeof64) {
            bytes.push_back(0x00);
        }
        //reverses byte values since the command needs them reversed. The reversed function

        command.insert(command.end(), bytes.begin(), bytes.end());
    }

    if (useDps == 1) {
        std::stringstream ss;
        ss << std::hex << dps;
        string hexString = ss.str();
        if ((hexString.length() % 2 != 0)/* && hexString.length() != 16*/) {
            hexString = "0" + hexString;
        }
        //Converts string values into hex
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i < hexString.length(); i += 2) {
            std::string byteString = hexString.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(std::stoi(byteString, nullptr, 16));
            bytes.push_back(byte);
        }

        //reverses byte values since the command needs them reversed. The reversed function
        std::reverse(bytes.begin(), bytes.end());

        while (bytes.size() < sizeof32) {
            bytes.push_back(0x00);
        }

        command.insert(command.end(), bytes.begin(), bytes.end());

    }

    if (useTorque == 1) {
        std::stringstream ss;
        ss << std::hex << degreePosition;
        string hexString = ss.str();
        if ((hexString.length() % 2 != 0)/* && hexString.length() != 16*/) {
            hexString = "0" + hexString;
        }
        //Converts string values into hex
        std::vector<uint8_t> bytes;
        for (size_t i = 0; i < hexString.length(); i += 2) {
            std::string byteString = hexString.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(std::stoi(byteString, nullptr, 16));
            bytes.push_back(byte);
        }
        //reverses byte values since the command needs them reversed. The reversed function
        std::reverse(bytes.begin(), bytes.end());
        command.insert(command.end(), bytes.begin(), bytes.end());
    }
    command.push_back(0x00);
    store = 0;
    //For loops insert Checsum
    for (int i = 5; i < CMDSize; i++) {
        store += command[i];
    }

    if (numCheck > 1) {
        command[checksumLoc[1]] = store;
    }


    write(serial, buffer(command.data(), command.size()));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "leg1p3.1 PASS");
}

std::vector<double> readResponse(boost::asio::serial_port& serial, std::vector<uint8_t> buf, boost::system::error_code ec) {
    std::stringstream ss;
    size_t len = serial.read_some(buffer(buf.data(), buf.size()), ec);
    std::vector<uint8_t> data;
    if (ec)
    {
        // Handle any error
        std::cerr << "Error on receive: " << ec.message() << std::endl;
    }
    else
    {
        std::cout << "Received " << len << " bytes: ";


        // Print each byte in hex and store it in the vector
        for (size_t i = 0; i < len; ++i)
        {
            uint8_t byte = static_cast<uint8_t>(buf[i]); // Cast each byte to uint8_t
            data.push_back(byte);                        // Add the byte to the vector
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }

        std::cout << std::endl;

        // Now `data` contains the bytes received as uint8_t, and you can use it for further processing
    }


    // Logic for RX MOTOR STATE 2
    // [0]=Frame Head [1]=Command [2]=ID [3]=Data Length [4]=Checksum
    // [5]=Temp [6-7]=torque [8-9]=MotorSpeed [10-11]=Encoder Position [12] Checksum

    //Motor Temperature/Torque Current/MotorSpeed/EncoderPosition
    vector<double> rx = { 0,0,0,0 };
    //vector<uint8_t> responseExample = {0x3E,0x9C,0x01,0x07,0x20,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A};
    //Reads the Motor ID of the responding device
    //Reads the position of the responding device
    //Takes Hex values and converts them to the appropriate 
    std::string concatenatedHex;
    for (size_t i = 0; i < commandInfoRX[0].size(); ++i) {
        ss.str(""); // Clear the stringstream
        std::string startIndex = std::get<0>(commandInfoRX[0][i]); // Index of the first element to concatenate
        string unit = std::get<1>(commandInfoRX[0][i]); // Index of the first element to concatenate
        int startIndexValue = std::get<2>(commandInfoRX[0][i]); // Index of the first element to concatenate
        int maxIndex = std::get<3>(commandInfoRX[0][i]); // Maximum index to concatenate up to

        // Reset stringstream and concatenate hex values
        for (int j = maxIndex; j >= startIndexValue && j < data.size(); --j) {
            ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(data[j]);
        }

        concatenatedHex = ss.str();
        rx[i] = std::stoi(concatenatedHex, nullptr, 16);
        cerr << startIndex << unit << rx[i] << endl;
    }

    // Convert the concatenated hexadecimal string to an integer
    return rx;
}
