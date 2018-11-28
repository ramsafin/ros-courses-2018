#include "wander_bot/ForwardRotateExplorer.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "fwd_rot_explorer");

    // Create new stopper object
    ForwardRotateExplorer explorer;

    // Start the movement
    explorer.startMoving();

    return 0;
}
