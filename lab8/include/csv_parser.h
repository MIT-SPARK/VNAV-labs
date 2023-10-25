#pragma once

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

class CSVRow {
 public:
  std::string const& operator[](std::size_t index) const { return m_data[index]; }
  std::size_t size() const { return m_data.size(); }
  void readNextRow(std::istream& str) {
    std::string line;
    std::getline(str, line);

    // strip line of carriage returns '\r'
    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

    std::stringstream lineStream(line);
    std::string cell;

    m_data.clear();
    while (std::getline(lineStream, cell, ',')) {
      m_data.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty()) {
      // If there was a trailing comma then add an empty element.
      m_data.push_back("");
    }
  }

 private:
  std::vector<std::string> m_data;
};

inline std::istream& operator>>(std::istream& str, CSVRow& data) {
  data.readNextRow(str);
  return str;
}

class CSVIterator {
 public:
  typedef std::input_iterator_tag iterator_category;
  typedef CSVRow value_type;
  typedef std::size_t difference_type;
  typedef CSVRow* pointer;
  typedef CSVRow& reference;

  CSVIterator(std::istream& str) : m_str(str.good() ? &str : NULL) { ++(*this); }
  CSVIterator() : m_str(NULL) {}

  // Pre Increment
  CSVIterator& operator++() {
    if (m_str) {
      if (!((*m_str) >> m_row)) {
        m_str = NULL;
      }
    }
    return *this;
  }
  // Post increment
  CSVIterator operator++(int) {
    CSVIterator tmp(*this);
    ++(*this);
    return tmp;
  }
  CSVRow const& operator*() const { return m_row; }
  CSVRow const* operator->() const { return &m_row; }

  bool operator==(CSVIterator const& rhs) {
    return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
  }
  bool operator!=(CSVIterator const& rhs) { return !((*this) == rhs); }

 private:
  std::istream* m_str;
  CSVRow m_row;
};
