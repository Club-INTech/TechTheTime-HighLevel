//Main
//Importer les trucs
//Créer les fonctions
//Envoyer les instructions à controller
//Binder le tout
//Tester avec Tim
//#include "client/ActionClient.hpp"
#include "controller/controllerSetup.hpp"
#include "order/order.hpp"

using namespace order;

int main(void) {
    Binder binder;

    binder.BindButton(0, [](int button, bool state) {
        if (state) {
            goTo(0, 0);
        }
    });
    binder.BindAxis(0, joystickMove);
    binder.BindAxis(1, joystickMove);

    ControllerSetup controller = ControllerSetup(binder);
    controller.run(false);
}