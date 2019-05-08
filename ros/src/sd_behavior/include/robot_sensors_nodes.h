#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>

#define VENTOUSE_GAUCHE 0
#define VENTOUSE_DROITE 2
#define VENTOUSE_CENTRE 1
#define PALET_GAUCHE 8
#define PALET_CENTRE 9
#define PALET_DROIT 10
#define TIRRETTE_DEMARAGE 7

/*
#### ros topic "/r1/pilo/switches" : Uint32
description des bit du topic :
bits   : fils                   : connection arduino    : description
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
NC

 rostopic echo /r1/pilo/switches
*/
namespace RobotSensors
{
	BT::NodeStatus IsTirettePresente();
	BT::NodeStatus IsCampViolet();

	BT::NodeStatus IsPaletDroit();
	BT::NodeStatus IsPaletCentre();
	BT::NodeStatus IsPaletGauche();

	BT::NodeStatus IsVentouseDroit();
	BT::NodeStatus IsVentouseCentre();
	BT::NodeStatus IsVentouseGauche();
	void rosUpdateSwitch(const std_msgs::UInt32 &switches);
}

