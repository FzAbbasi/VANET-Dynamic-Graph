

#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/modules/messages/Ack_Msg_m.h"
#include "veins/modules/messages/TokenMsg_m.h"
#include "veins/base/utils/SimpleAddress.h"
#include <vector>
#include <map>
#include <string>

namespace veins {

constexpr LAddress::L2Type L2BROADCAST = 0xFFFFFFFF;

class VEINS_API TraCIDemo11p : public DemoBaseApplLayer {


public:
    struct Token {
        int index;
        int vehicleId;
        Coord position; // Use Coord instead of std::string for position
        LAddress::L2Type destination;

    };



public:
    // Define time interval
    const double TIME_INTERVAL = 5.0; // 5 seconds


    int Nod_Id ;
    Coord Node_Pos ;
    double transmissionRate;
    int bestLinkId;
    double maxEfficiency;


    std::vector<std::pair<Coord, int>> graphSnapshots;
    std::vector<std::pair<Coord, int>> currentSnapshot; 
    std::vector<std::pair<int, double>> ConnectivityScores;
    std::vector<std::vector<int>> adjacency_List; 
    std::vector<std::pair<int, double>> efficiency_vec; 
    std::vector<std::pair<int, double>> efficiency;
    std::vector<std::vector<int>> best_efficiency; 


public:
    void initialize(int stage) override;
    void handleSelfMsg(cMessage* msg) override;
    void handleLowerMsg(cMessage* msg) override;
    void onBSM(DemoSafetyMessage* bsm) override;

    std::vector<std::pair<Coord,int>> createGraphSnapshot(int Id, Coord Position);
    std::vector<std::pair<int, double>> calculateDistanceCentrality(const std::vector<std::pair<Coord, int>>& currentsnapshot);
    std::vector<std::pair<int, double>> calculateEfficiency(int Nod_Id,const std::vector<std::pair<int, double>>& ConnectivityScores);
    double calculateTransmissionRate(int nodeId, int ConnectivityScores_id);
    double calculateTransmissionDelay(int nodeId, int neighborId, double transmissionRate);
    Coord getposition(int id);
    void sendTokensBasedOnEfficiency(const std::vector<std::pair<int, double>>& efficiency);
    std::vector<TraCIDemo11p::Token> createTokens(DemoSafetyMessage* bsm, int tokenSizeThreshold);
    int selectBestLinkForToken(const TraCIDemo11p::Token& token,const std::vector<std::pair<int, double>>& efficiency );
    void sendToken(TraCIDemo11p::Token token , int id_distination);
    void onTokenReceived(TokenMsg* tokenMsg);


};

} // namespace veins
