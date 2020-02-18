#ifndef _name_h_
#define _name_h_
#include <unordered_map>
class NameRecords {
    std::unordered_map<std::string, uint8_t> name2id;
    std::unordered_map<uint8_t, std::string> id2name;
public:    
    NameRecords() {
        name2id["base_station"] = 0;
        id2name[0] = "base_station";
        name2id["robot_1"] = 1;
        id2name[1] = "robot_1";
        name2id["robot_2"] = 2;
        id2name[2] = "robot_2";
        name2id["robot_3"] = 3;
        id2name[3] = "robot_3";
        name2id["robot_4"] = 4;
        id2name[4] = "robot_4";
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
            return std::to_string(id);
    }
};

#endif