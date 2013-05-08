#include <string>

/** Variables holding the settings extracted from the command line. */

extern std::string  gHostname;
extern unsigned     gPort;
extern unsigned     gIndex;
extern unsigned     gDebug;
extern unsigned     gFrequency;
extern unsigned     gDataMode;
extern bool         gUseLaser;

/** Parses the cmd line, putting the data in the above variables. */
int parse_args(int argc, char** argv);
