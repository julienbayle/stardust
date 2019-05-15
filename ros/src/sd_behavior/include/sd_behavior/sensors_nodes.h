#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/UInt32.h>

/*
#### ros topic "/r1/pilo/switches" : Uint32
description des bit du topic :
bits   : fils                   : onnection arduino     : description
bit 0  : fils bleu/noir_rouge 	:Pilo:P_ANA1.Pin.IO0 	: ventouse gauche enforcé
bit 1  : noir_bleu            	:Pilo:P_ANA1.Pin.IO1 	: ventouse centre enforcé
bit 2  : noir_vert 			  	:Pilo:P_ANA1.Pin.IO2 	: ventouse droite enforcé
bit 3  : NC
bit 4  : banc                  	:Pilo:P_ANA1.Pin.IO5	: (vbat div) 
bit 5  : fils bleu 				:Pilo:P_ANA1.Pin.IO5 	: (gnd_power)
bit 6  : fils vert				:Pilo:P_ANA1.Pin.IO6 	: bouton arret d'urgence no percuté(robot alimenté)
bit 7  : fils jaune				:PiloP_ANA1.Pin.IO7 	: tirette de demarrage presente
bit 8  : fils noir/bleu_rouge	:PiloP_COM3.Pin.IO0 	: palet gauche present
bit 9  : fils noir_bleu			:PiloP_COM3.Pin.IO1 	: palet centre present
bit 10 : fils noir_vert			:PiloP_COM3.Pin.IO5 	: palet droite present
*/

#define VENTOUSE_GAUCHE 0
#define VENTOUSE_CENTRE 1
#define VENTOUSE_DROITE 2
#define ARRET_URGENCE 6
#define TIRRETTE_DEMARAGE 7
#define PALET_GAUCHE 8
#define PALET_CENTRE 9
#define PALET_DROIT 10

namespace SensorsNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory);
	
	BT::NodeStatus IsArretUrgence();
	BT::NodeStatus IsTirettePresente();
	BT::NodeStatus IsCampViolet();
	BT::NodeStatus IsRobotEnMouvement();

	BT::NodeStatus IsPaletDroit();
	BT::NodeStatus IsPaletCentre();
	BT::NodeStatus IsPaletGauche();

	BT::NodeStatus IsVentouseDroite();
	BT::NodeStatus IsVentouseCentre();
	BT::NodeStatus IsVentouseGauche();

	void rosUpdate(const std_msgs::UInt32 &switches);
}
