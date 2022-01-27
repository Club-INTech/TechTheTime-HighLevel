//Main
//Importer les trucs
//Créer les fonctions
//Envoyer les instructions à controller
//Binder le tout
//Tester avec Tim
//#include "client/ActionClient.hpp"
#include "controller/controllerSetup.hpp"
#include "order/order.hpp"
#include "client/ActionClient.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include "action_msg_srv/srv/order.hpp"

using namespace order;

int main(void) {
    Binder binder;
    auto c = new ActionClientNode<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, std::string, int64_t, int64_t, int64_t, int64_t>("client_node");
    binder.BindButton(0, [](int button, bool state) {
        if (state) {
            goTo(0, 0);
        }
    });
    binder.BindAxis(0, joystickMove);
    binder.BindAxis(1, joystickMove);

    ControllerSetup controller = ControllerSetup(binder);
    controller.run(false);
    return 0;
}