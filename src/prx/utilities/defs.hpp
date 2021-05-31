#include "prx/utilities/general/prx_assert.hpp"
#include "prx/utilities/general/constants.hpp"
#include "prx/utilities/general/random.hpp"
#include "prx/utilities/general/transforms.hpp"
#include "prx/utilities/general/string_manip.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/zipped_iter.hpp"

#define PRX_DEBUG_PRINT printf("%s: %d\n", __PRETTY_FUNCTION__, __LINE__ );
#define PRX_DEBUG_ITERABLE(msg, v) std::cout << "[DBG " << msg << "] "; for(auto e : v){std::cout << e << " ";}std::cout << std::endl;

#define STR_TO_BOOL(VAL) (std::string(VAL)=="True" | std::string(VAL) == "true")
#define STR_TO_INT(VAR) (std::stoi(VAR))
#define STR_TO_DOUBLE(VAR) (std::stod(VAR))

/**
 * Convert VAR to int and test VAR against CHECK. Eg. STR_TO_INT_AND_CHECK(foo, >= 0) ==> if(foo >= 0): parse(foo) else: THROW 
 * @param  VAR      String to parse to int
 * @param  CHECK    Check to perform
 * @return          The parse value or throws error if check is not passed.
 */
#define STR_TO_INT_AND_CHECK(VAR, CHECK) STR_TO_INT(VAR) CHECK ? STR_TO_INT(VAR) : throw prx_assert_t(#CHECK, __FILE__, __LINE__, (prx_assert_t::stream_t() << "STR TO INT - CHECK not passed!"))
#define STR_TO_DOUBLE_AND_CHECK(VAR, CHECK) STR_TO_DOUBLE(VAR) CHECK ? STR_TO_DOUBLE(VAR) : throw prx_assert_t(#CHECK, __FILE__, __LINE__, (prx_assert_t::stream_t() << "STR TO DOUBLE - CHECK not passed!"))
