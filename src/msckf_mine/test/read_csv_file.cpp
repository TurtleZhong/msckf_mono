#include "common_include.h"
#include "csv.h"
#include "config.h"

using namespace MSCKF_MINE;

int main(int argc, char *argv[])
{
    /*test the config file*/
    Config::setParameterFile("../config/config.yaml");
    string sequence_dir = Config::get<string>("sequence_dir");
    cout << "sequence_dir = " << sequence_dir << endl;

    /*test the csv.h file*/

    return 0;
}

