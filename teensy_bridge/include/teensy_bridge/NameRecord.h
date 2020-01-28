#ifndef _name_h_
#define _name_h_
#include <unordered_map>
class NameRecords {
    std::unordered_map<std::string, uint8_t> name2id;
    std::unordered_map<uint8_t, std::string> id2name;
public:    
    NameRecords() {
    }
    void addNameRecord(std::string name, uint8_t id) {
        name2id[name] = id;
        id2name[id] = name;
    }
    uint8_t getId(std::string name){
        //TODO: NAME NOT FOUND EXCEPTION
        return name2id[name];
    }
    std::string getName(uint8_t id){
        if(id2name.count(id) > 0)
            return id2name[id];
        else
            return "";
    }
};

#endif