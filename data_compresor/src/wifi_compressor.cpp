#include <data_compressor/msgs/WifiArray.h>
#include <data_compressor/protocol.h>
#include <data_compressor/parser.h>
#include <data_compressor/zip.h>

wireless_msgs::WifiArray toWifiArray(WifiArray wa) {
    wireless_msgs::WifiArray msg;
    msg.header.frame_id = "wifi";
    msg.header.stamp = ros::Time(wa.timestamp/1000000000ull, wa.timestamp);
    msg.position.x = wa.x;
    msg.position.y = wa.y;
    msg.position.z = wa.z;
    for(int i=0; i < wa.length; i++) {
        wireless_msgs::Wifi wmsg;
        wmsg.ssid.data = wa.data[i].ssid;
        wmsg.signal.data = wa.data[i].signal;
        wmsg.quality.data = wa.data[i].quality;
        msg.data.push_back(wmsg);
    }
    return msg;
}

std::vector<uint8_t> encodeWifiArray(WifiArray wa) {
    std::vector<uint8_t>  bytestream;
    encodeUInt8t(bytestream, (uint8_t)MessageType::WIFI_SIGNAL); //1
    encodeUInt64t(bytestream, wa.timestamp); //9
    encodeInt(bytestream, wa.x); //12
    encodeInt(bytestream, wa.y); //16
    encodeInt(bytestream, wa.z); //20
    encodeUInt8t(bytestream, wa.length); //21
    for (int i=0; i<wa.length; i++) {
        encodeWifi(bytestream, wa.data[i]); //???
    }
    return bytestream;
}

void encodeWifi(std::vector<uint8_t>& bytestream, Wifi w) {
    encodeString(bytestream, w.ssid);
    encodeString(bytestream, w.signal);
    encodeString(bytestream, w.quality);
}

WifiArray decodeWifiArray(std::vector<uint8_t> packet) {
    PacketParser state;
    WifiArray wa;
    uint8_t type = expectInt8t(state, packet);
    wa.timestamp = expectUInt64t(state, packet);
    wa.x = expectInt(state, packet);
    wa.y = expectInt(state, packet);
    wa.z = expectInt(state, packet);
    wa.length = expectUInt8t(state, packet);
    for (int i=0; i<wa.length; i++) {
        wa.data.push_back(decodeWifi(state, packet));
    }
    return wa;
}

Wifi decodeWifi(PacketParser& state, std::vector<uint8_t>& packet) {
    Wifi w;
    w.ssid = expectString(state, packet);
    w.signal = expectString(state, packet);
    w.quality = expectString(state, packet);
    return w;
}

wireless_msgs::LoraPacket toLoraPacket(WifiArray wa) {
    wireless_msgs::LoraPacket packet;
    packet.to.data = "base_station";
    packet.data = compressZip(encodeWifiArray(wa));
}

wireless_msgs::LoraPacket toLoraPacket(wireless_msgs::WifiArray array){
    WifiArray out_array;
    out_array.x = array.position.x;
    out_array.y = array.position.y;
    out_array.z = array.position.z;
    for(int i = 0; i < array.data.size(); i++){
        Wifi w;
        w.ssid = array.data[i].ssid.data;
        w.quality = array.data[i].quality.data;
        w.signal = array.data[i].signal.data;
        out_array.data.push_back(w);
    }
    return toLoraPacket(out_array);
}

