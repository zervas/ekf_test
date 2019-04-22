// Copyright
//
// TODO

#ifndef EKF_TEST_READ_CSV_H_
#define EKF_TEST_READ_CSV_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class CSVRow {
    std::vector<std::string> m_data;

 public:
    std::string const & operator[](std::size_t index) const {
        return m_data[index];
    }

    void read_next_row(std::istream &in) {
        std::string line;
        std::getline(in, line);  // get 'char's from stream until '\n'
        std::stringstream lineStream(line);
        std::string cell;

        m_data.clear();
        while (std::getline(lineStream, cell, ' ')) {
            m_data.push_back(cell);
        }

        // Check for trainling comma with no data after it
        if (!lineStream && cell.empty()) {
            m_data.push_back("No more ");
        }
    }
};

#endif  // EKF_TEST_READ_CSV_H_
