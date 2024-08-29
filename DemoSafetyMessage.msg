
import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;

namespace veins;

packet DemoSafetyMessage extends BaseFrame1609_4 {   
    Coord SenderPos;
    Coord SenderSpeed;
    simtime_t SendingTim;
    int Id_veh;    
    int ByteLength = 300;
}
