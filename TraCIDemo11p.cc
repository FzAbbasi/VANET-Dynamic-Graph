

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/base/utils/SimpleAddress.h"
#include "veins/modules/messages/TokenMsg_m.h"
#include "veins/modules/messages/Ack_Msg_m.h"


using namespace veins;

Define_Module(veins::TraCIDemo11p);


void TraCIDemo11p::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        maxEfficiency = 0.0;
        scheduleAt(simTime() + TIME_INTERVAL, new cMessage("snapshotTimer"));
    }
}


void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    if (msg->isSelfMessage() && std::string(msg->getName()) == "snapshotTimer") {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        bsm->setSenderPos(mobility->getPositionAt(simTime()));
        bsm->setSenderSpeed(mobility->getHostSpeed());
        bsm->setSendingTim(simTime());
        bsm->setId_veh(mobility->getId());
        sendDown(bsm);
        EV << "Sent BSM from vehicle ID: " << mobility->getId() << endl;
        scheduleAt(simTime() + TIME_INTERVAL, msg); // Reschedule for the next snapshot

    }else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}


void TraCIDemo11p::handleLowerMsg(cMessage* msg) {
    if (TokenMsg* tokenMsg = dynamic_cast<TokenMsg*>(msg)) {
            onTokenReceived(tokenMsg);
        }else {
            // Handle other message types
            DemoBaseApplLayer::handleLowerMsg(msg);
        }
}

void TraCIDemo11p::onBSM(DemoSafetyMessage* bsm) {
    Nod_Id = bsm->getId_veh();
    Node_Pos = bsm->getSenderPos();
    graphSnapshots = createGraphSnapshot(Nod_Id,Node_Pos);
    // Check connectivity
    if (graphSnapshots.size() > 1) {
        std::vector<std::pair<int, double>> Connectivity_Scores = calculateDistanceCentrality(graphSnapshots);
        if (!ConnectivityScores.empty() || (!adjacency_List.empty())){
            std::vector<std::pair<int, double>> efficiency = calculateEfficiency(Nod_Id, ConnectivityScores);
            if (!efficiency.empty()){
                sendTokensBasedOnEfficiency(efficiency);
            }else{
                EV << "ConnectivityScores is empty." << endl;
                scheduleAt(simTime() + TIME_INTERVAL, new cMessage("snapshotTimer"));
            }
        }else{
            EV << "ConnectivityScores is empty." << endl;
            scheduleAt(simTime() + TIME_INTERVAL, new cMessage("snapshotTimer"));
        }
    } else {
        EV << "Not enough nodes to determine connectivity." << endl;
        scheduleAt(simTime() + TIME_INTERVAL, new cMessage("snapshotTimer"));
    }
}


std::vector<std::pair<Coord, int>> TraCIDemo11p::createGraphSnapshot(int Id, Coord Position) {
    int index =0;

    currentSnapshot.clear();
    adjacency_List.clear();
    //all vehicles
    std::map<std::string, cModule*> availableCars = mobility->getManager()->getManagedHosts();
    adjacency_List.resize(availableCars.size()); // Resize to the number of vehicles

    std::map<std::string, cModule*>::iterator it;
    for (it = availableCars.begin(); it != availableCars.end(); it++){
        TraCIMobility* mobility = TraCIMobilityAccess().get(it->second);
        //finding one-hop nodes
        int current_Id = mobility->getId();
        Coord current_pos = mobility->getPositionAt(simTime());
        if (current_Id != Id) {
            double distance = Position.distance(current_pos);
            if (distance < 100.0) {
                currentSnapshot.push_back({current_pos, current_Id});
                adjacency_List[index].push_back(current_Id); // Add neighbor ID to the current vehicle's adjacency list
                }
            }
        index++;
        }
    // Store the current snapshot
    return currentSnapshot;
}


std::vector<std::pair<int, double>> TraCIDemo11p::calculateDistanceCentrality(const std::vector<std::pair<Coord, int>>& currentsnapshot) {
    double totalDistance = 0.0;
    int count = 0;
    double score;
    ConnectivityScores.clear();

    for (const auto& neighbor : currentsnapshot) {
        int neighborId = neighbor.second; // Get the neighbor ID
        Coord neighborPosition = neighbor.first; // Get the neighbor position

        double distance = 0.0;
        int count = 0;

        // calculate distance with all other nodes in the current snapshot
        for (const auto& neighborPair : currentSnapshot) {
            if (neighborId != neighborPair.second){
                double distance = neighborPosition.distance(neighborPair.first);
                if (distance > 0) {
                    totalDistance += distance;
                    count++;
                }
            }
        }
        if (count > 0){
            score = count / totalDistance;
            ConnectivityScores.push_back({neighborId, score});
        }else{
            score = 0;
            ConnectivityScores.push_back({neighborId, score});
        }
    }
    // Sort the scores in descending order
    std::sort(ConnectivityScores.begin(), ConnectivityScores.end(),
               [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                   return a.second > b.second; // Sort by score in descending order
               });

    return ConnectivityScores;
}


std::vector<std::pair<int, double>> TraCIDemo11p::calculateEfficiency(int Nod_Id,const std::vector<std::pair<int, double>>& ConnectivityScores){
    double efficiency_value;
    std::cout<<"this calculateEfficiency class"<< std::endl;
    efficiency_vec.clear();


    for (size_t i = 0; i < ConnectivityScores.size(); ++i) {
        int ConnectivityScores_id = ConnectivityScores[i].first;
        // Calculate the transmission rate and delay for the link between nodeId and neighborId
        double transmissionRate = calculateTransmissionRate(Nod_Id, ConnectivityScores_id);
        double transmissionDelay = calculateTransmissionDelay(Nod_Id, ConnectivityScores_id, transmissionRate);
        // Efficiency can be defined as the ratio of transmission rate to delay
        if (transmissionDelay > 0) {
            efficiency_value = transmissionRate / transmissionDelay;
            efficiency_vec.push_back({ConnectivityScores_id, efficiency_value});
            std::cout<<"this efficiency_value "<<efficiency_value<<std::endl;

        } else {
            efficiency_value = 0; // If delay is zero, return zero efficiency
            efficiency_vec.push_back({ConnectivityScores_id, efficiency_value});

            }
    }
    return efficiency_vec;
}


double TraCIDemo11p::calculateTransmissionRate(int nodeId, int ConnectivityScores_id) {
    std::cout<<"this calculateTransmissionRate class"<< std::endl;
    // Example: Assume a fixed maximum transmission rate (e.g., 1 Mbps)
    double maxTransmissionRate = 1.0e6; // 1 Mbps
    transmissionRate=0;

    Coord getting_pos = getposition(ConnectivityScores_id);

    // Get the distance between the nodes
    double distance = Node_Pos.distance(getting_pos);

    // If the distance is more than the maximum communication range, return 0
    if (distance > 100.0) {
        transmissionRate = 0.0;
    }else{
        // Calculate the transmission rate based on distance
        transmissionRate = maxTransmissionRate * (1 - (distance / 100.0)); // Linear decrease
        std::cout<<"TransmissionRate is"<<transmissionRate<< std::endl;

        if (transmissionRate < 0){
            transmissionRate = 0;
            std::cout<<"TransmissionRate is"<<transmissionRate<< std::endl;

        }
    }
    return transmissionRate;
}


double TraCIDemo11p::calculateTransmissionDelay(int nodeId, int neighborId, double transmissionRate) {
    // Assume a fixed data size (e.g., 1 KB)
    double dataSize = 1024; // 1 KB
    std::cout<<"calculateTransmissionDelay calss"<< std::endl;

    // Calculate the transmission delay based on the data size and transmission rate
    return (transmissionRate > 0) ? (dataSize / transmissionRate) : std::numeric_limits<double>::max(); // Delay in seconds
}


Coord TraCIDemo11p::getposition(int id){
    std::cout<<"getposition calss"<< std::endl;

    Coord current_pos;
    //all vehicles
    std::map<std::string, cModule*> availableCars = mobility->getManager()->getManagedHosts();
    std::map<std::string, cModule*>::iterator it;
    for (it = availableCars.begin(); it != availableCars.end(); it++){
        TraCIMobility* mobility = TraCIMobilityAccess().get(it->second);
        //finding one-hop nodes
        int current_Id = mobility->getId();
        if (id == current_Id){
            current_pos = mobility->getPositionAt(simTime());
        }
    }
    return current_pos;
}


void TraCIDemo11p::sendTokensBasedOnEfficiency(const std::vector<std::pair<int, double>>& efficiency) {

    //Create a new DemoSafetyMessage for broadcasting
    DemoSafetyMessage* bsm = new DemoSafetyMessage();

    //Initialize bsm fields as needed
    bsm->setSenderPos(mobility->getPositionAt(simTime()));
    bsm->setSenderSpeed(mobility->getHostSpeed());
    bsm->setSendingTim(simTime());
    bsm->setId_veh(mobility->getId());

   // Creating a vector to hold tokens
   std::vector<Token> tokens_vec;

   //  if it exceeds a certain size => Tokenize the message
   int tokenSizeThreshold = 256;
   if (bsm->getByteLength() >tokenSizeThreshold ) {
       tokens_vec = createTokens(bsm, tokenSizeThreshold);

       for (const auto& token : tokens_vec) {
           int selectedLinkId = selectBestLinkForToken(token,efficiency); // Select the best link for the current token
          // token.destination = selectedLinkId;
           sendToken(token,selectedLinkId); // Send the token over the selected link
       }
   }else {
       // Send a single token if it's small
       std::vector<Token> tokens = createTokens(bsm, tokenSizeThreshold); // Create tokens using the helper function
       if (!tokens.empty()) { // Check if at least one token was created
           Token singleToken = tokens[0];
           int selectedLinkId = selectBestLinkForToken(singleToken,efficiency); // Select the best link for the single token
          // token.disti = selectedLinkId;
           sendToken(singleToken, selectedLinkId); // Send the single token over the selected link
       }
   }
}



// create tokens from a DemoSafetyMessage
std::vector<TraCIDemo11p::Token> TraCIDemo11p::createTokens(DemoSafetyMessage* bsm, int tokenSizeThreshold)  {
    std::vector<TraCIDemo11p::Token> tokens_vec;
    int number_of_Tokens = (bsm->getByteLength() + tokenSizeThreshold - 1) / tokenSizeThreshold;

    for (int i = 0; i < number_of_Tokens; ++i) {
        TraCIDemo11p::Token token;
        token.index = i;
        token.vehicleId = bsm->getId_veh();
        token.position = bsm->getSenderPos();
        //Using L2BROADCAST as the destination for token to receive message by all nodes within the broadcast range.
        //token.destination = L2BROADCAST;
        tokens_vec.push_back(token);
    }

    return tokens_vec;
}


int TraCIDemo11p::selectBestLinkForToken(const TraCIDemo11p::Token& token,const std::vector<std::pair<int, double>>& efficiency ) {

    for (int i = 0; i < efficiency.size(); ++i) {
        // Check if this efficiency is the highest we've seen
        if (efficiency[i].second > maxEfficiency) {
            maxEfficiency = efficiency[i].second;//store value of efficiency
            bestLinkId = efficiency[i].first; // Store ID
        }
    }
    return bestLinkId; // Return the best link ID or -1 if not found
}


void TraCIDemo11p::sendToken(TraCIDemo11p::Token token , int id_distination) {
    // Create a new TokenMsg to send the token
    TokenMsg* tokenMsg = new TokenMsg();

    // Set the fields of the TokenMsg
    tokenMsg->setTokenId(token.index);
    tokenMsg->setMessageId(token.vehicleId);
    tokenMsg->setDest_Address(id_distination); // Set the destination address

    EV << "Sent token : " << token.index << endl;


    // Send the token message
    sendDelayedDown(tokenMsg, 0.001); // Send with a slight delay

}

void TraCIDemo11p::onTokenReceived(TokenMsg* tokenMsg) {
    int tokenId = tokenMsg->getTokenId();
    int messageId = tokenMsg->getMessageId();
    int destAddress = tokenMsg->getDest_Address();

    // Check if the received token is intended for this vehicle
    if (destAddress == mobility->getId()) {
        EV << "Received token: " << tokenId << " from vehicle: " << messageId << endl;
        // Process the received token as needed
        // ...
    } else {
        EV << "Received token: " << tokenId << " from vehicle: " << messageId << " but it's not for me." << endl;
    }

    // Clean up the token message
    delete tokenMsg;
}

