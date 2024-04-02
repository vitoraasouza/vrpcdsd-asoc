#include "Infos/InputSettings.h"
#include "MathModel/CompactModel.h"
#include "Persistent/Instance.h"

int main(int argc, char * argv[]) {

    InputSettings input_settings;
    if (!input_settings.parseSettings(argc, argv)) {
        exit(1);
    }

    Instance::Init(input_settings.instance_file_name_and_dir_);

    CompactModel mip(input_settings);
    mip.solve();

    if (input_settings.only_relaxation_) {
        mip.printTikZRelaxationGuide(input_settings.tikz_relaxation_file_name_);
    } else {
        mip.printSolutionOnFile(input_settings.solution_file_name_);
        mip.printTikZSolutionGuide(input_settings.tikz_file_name_);
    }
    mip.printReport();

    return 0;

}
