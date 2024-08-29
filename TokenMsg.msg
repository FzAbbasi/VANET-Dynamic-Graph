
import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;

namespace veins;

// TokenMsg extends BaseFrame1609_4
packet TokenMsg extends BaseFrame1609_4
{  
    int Dest_Address;
    int MessageId;
    int TokenId;
}
