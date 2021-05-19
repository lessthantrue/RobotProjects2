#ifndef YAML_CONFIG_READER_H
#define YAML_CONFIG_READER_H
 
// mostly copied from the rviz implementation

#include <yaml-cpp/yaml.h>

using std::string;

class YamlConfigReader{
public:
    void readFile(const string& fname);
};

#endif