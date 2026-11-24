/* PC test for the barcode character -> direction table (barcode_algo.c, bd_map_*), set from the dashboard over MQTT.
 *   make map        (separate from test_s3.c, which no longer matches the barcode API) */
#include <stdio.h>
#include "../../../app_program/subsystem3/barcode_algo.h"
#include "../../../app_program/subsystem5/vc_msgs.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
	/* start-up defaults from line_config.h */
	CHECK(bd_map_get('A') == NAV_LEFT && bd_map_get('B') == NAV_RIGHT && bd_map_get('C') == NAV_STRAIGHT &&
	      bd_map_get('D') == NAV_UTURN, "defaults A-D");
	CHECK(bd_map_get('E') == -1 && bd_map_get('7') == -1 && bd_map_get('*') == -1, "others have no command");

	unsigned v0 = bd_map_version();
	CHECK(bd_map_set('A', NAV_RIGHT) && bd_map_set('e', NAV_STOP) && bd_map_set('7', NAV_UTURN) && bd_map_set('D', -1),
	      "set A, e (lower case), 7, clear D");
	CHECK(bd_map_get('A') == NAV_RIGHT && bd_map_get('E') == NAV_STOP && bd_map_get('7') == NAV_UTURN &&
	      bd_map_get('D') == -1, "new directions used");
	CHECK(bd_map_version() != v0, "version changes so S1 republishes");
	CHECK(!bd_map_set('*', NAV_LEFT) && !bd_map_set('A', 9) && !bd_map_set('A', -2), "bad character / direction refused");
	CHECK(bd_map_get('A') == NAV_RIGHT, "refused set changes nothing");

	int8_t tab[BD_MAP_N];
	bd_map_copy(tab);
	CHECK(tab[0] == NAV_RIGHT && tab[3] == -1 && tab[4] == NAV_STOP && tab[26 + 7] == NAV_UTURN && tab[25] == -1, "copy");
	CHECK(bd_map_char(0) == 'A' && bd_map_char(25) == 'Z' && bd_map_char(26) == '0' && bd_map_char(35) == '9' &&
	      bd_map_char(36) == 0 && bd_map_index('z') == 25 && bd_map_index('9') == 35, "index <-> character");

	bd_map_defaults();
	CHECK(bd_map_get('A') == NAV_LEFT && bd_map_get('E') == -1 && bd_map_get('D') == NAV_UTURN, "defaults restored");

	printf("%d checks, %d failed\n%s\n", checks, fails, fails ? "SOME FAILED" : "ALL PASSED");
	return fails != 0;
}
