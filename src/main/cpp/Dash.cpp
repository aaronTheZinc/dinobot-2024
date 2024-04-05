#include <Dash.h>
#include <iostream>
Dashboard::Dashboard() {
    dash = nt::NetworkTableInstance::GetDefault().GetTable("datatable");
    dash->PutNumber("@command", 0);
}

void Dashboard::periodic() {
    command = dash->GetNumber("@command", 0);
    std::cout << "COMMAND  " << command << std::endl;

};

int Dashboard::getCommand() {
    return this->command;
}

void Dashboard::commandCompleted(int cmd) {
    switch(cmd) {
        case 1: {
            dash->PutBoolean("@command/completed", true );
            return;
        }
    }
}