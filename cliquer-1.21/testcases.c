/*
 * Unit tests for cliquer.
 *
 * Run by "make test".
 */

#include <stdio.h>
#include <stdlib.h>

#include "cliquer.h"

#define N 16

int small_max_cliques[][N] = {
	{ 4,  0,2,4,7 },
	{ 0 }
};
int small_3_sized_cliques[][N] = {
	{ 4,  0,2,4,7 },
	{ 3,  2,4,7 },
	{ 3,  0,4,7 },
	{ 3,  0,2,7 },
	{ 3,  0,2,4 },
	{ 3,  0,2,5 },
	{ 3,  2,4,6 },
	{ 3,  1,4,6 },
	{ 0 }
};

int large_max_cliques[][N] = {
	{ 9,  6,29,152,284,378,388,409,561,594 },
	{ 9,  6,152,284,378,388,409,502,561,594 },
	{ 9,  12,24,62,159,205,231,312,423,509 },
	{ 9,  23,128,242,354,374,465,479,588,594 },
	{ 9,  35,46,196,264,297,307,378,476,541 },
	{ 9,  125,185,196,240,247,260,340,399,502 },
	{ 0 }
};

int large_w_max_cliques[][N] = {
	{ 8  ,5,167,365,368,446,479,540,574 },  /* w=66 */
	{ 7  ,137,148,208,219,453,460,542 },  /* w=66 */
	{ 0 }
};
int large_w_exact_62_cliques[][N] = {
        { 7  ,5,158,188,297,446,479,503 },  /* w=62 */
        { 7  ,9,170,251,297,396,401,523 },  /* w=62 */
        { 7  ,9,284,291,330,548,559,587 },  /* w=62 */
        { 7  ,9,284,291,330,558,559,587 },  /* w=62 */
        { 8  ,12,116,159,244,245,398,423,478 },  /* w=62 */
        { 7  ,20,194,264,269,347,458,515 },  /* w=62 */
        { 8  ,46,196,216,264,297,307,378,407 },  /* w=62 */
        { 7  ,52,102,120,152,244,348,499 },  /* w=62 */
        { 7  ,54,120,184,348,453,479,548 },  /* w=62 */
        { 8  ,55,102,131,152,293,333,500,551 },  /* w=62 */
        { 8  ,57,171,191,306,347,401,408,435 },  /* w=62 */
        { 7  ,62,133,142,173,400,416,587 },  /* w=62 */
        { 7  ,63,79,99,264,283,520,568 },  /* w=62 */
        { 8  ,70,108,293,330,343,514,548,588 },  /* w=62 */
        { 7  ,79,99,194,264,283,520,568 },  /* w=62 */
        { 7  ,90,175,226,291,400,401,440 },  /* w=62 */
        { 7  ,102,264,269,379,458,462,497 },  /* w=62 */
        { 7  ,127,132,235,242,354,398,578 },  /* w=62 */
        { 7  ,152,179,284,291,330,561,574 },  /* w=62 */
        { 7  ,155,160,236,332,420,436,556 },  /* w=62 */
        { 7  ,239,302,304,356,358,458,512 },  /* w=62 */
        { 7  ,239,304,313,356,358,458,512 },  /* w=62 */
        { 8  ,284,291,330,443,546,558,559,587 },  /* w=62 */
	{ 0 }
};

#include "testcase-large-over8.h"
#include "testcase-large-exact8.h"
#include "testcase-large-w-over60.h"
#include "testcase-large-w-60-64-mxml.h"

void test_single_clique(graph_t *g, int min_size, int max_size,
			boolean maximal, int (*sets)[N]);
void test_single_w_clique(graph_t *g,int min_size,int max_size,
			  boolean maximal, int (*sets)[N]);
void test_all_cliques(graph_t *g, int min_size, int max_size,
		      boolean maximal, int (*sets)[N]);
void test_all_w_cliques(graph_t *g, int min_size, int max_size,
			boolean maximal, int (*sets)[N]);
void test_user_function(graph_t *g, int min_size, int (*sets)[N]);
void test_too_small_array(graph_t *g,int min_size, int (*sets)[N]);
void test_max_weight(graph_t *g,int size);
void test_reentrance(graph_t *g1,int min1, int max1, boolean maximal1,
		     int (*sets1)[N], graph_t *g2,int min2,int max2,
		     boolean maximal2,int (*sets2)[N]);



int main(int argc, char *argv[]) {
	FILE *fp;
	graph_t *small, *large, *wlarge;


	/* No buffering on stdout */
	setvbuf(stdout, (char *)NULL, _IONBF, 0);

	printf("\n");
	printf("Running testcases:  ELEMENTSIZE=%d, sizeof(setelement)=%d  ",
	       ELEMENTSIZE,(int)sizeof(setelement));
	if ((sizeof(setelement)*8)!=ELEMENTSIZE) {
		printf("ERROR!\n");
		printf("Mismatch between ELEMENTSIZE and actual size of "
		       "setelement.\n");
		printf("Please reconfigure and recompile.\n");
		printf("\n");
		exit(1);
	}
	printf("(OK)\n\n");

	if (sizeof(unsigned long int) > sizeof(setelement)) {
		printf("NOTICE:  Size of unsigned long int is greater than "
		       "that of setelement.\n");
		printf("         You may wish to redefine setelement.\n\n");
	}

	if ((fp=fopen("testcase-small.a","rt"))==NULL) {
		perror("testcase-small.a");
		return 1;
	}
	small=graph_read_dimacs(fp);
	fclose(fp);
	if (!small) {
		return 1;
	}
	
	large=graph_read_dimacs_file("testcase-large.b");
	if (!large) {
		return 1;
	}

	if ((fp=fopen("testcase-large-w.b","rb"))==NULL) {
		perror("testcase-large-w.b");
		return 1;
	}
	wlarge=graph_read_dimacs(fp);
	fclose(fp);
	if (!wlarge) {
		return 1;
	}


	clique_default_options->time_function=NULL;

	printf("Testing small: graph_test...");
	if (!graph_test(small,NULL)) {
		printf("ERROR\n");
		graph_test(small,stdout);
		return 1;
	}
	printf("OK\n");
	printf("Testing large: graph_test...");
	if (!graph_test(large,NULL)) {
		printf("ERROR\n");
		graph_test(large,stdout);
		return 1;
	}
	printf("OK\n");
	printf("Testing wlarge: graph_test...");
	if (!graph_test(wlarge,NULL)) {
		printf("ERROR\n");
		graph_test(wlarge,stdout);
		return 1;
	}
	printf("OK\n");

	printf("\n");

	printf("Testing small: single maximum clique...");
	test_single_clique(small,0,0,FALSE,small_max_cliques);

	printf("Testing large: single maximum clique...");
	test_single_clique(large,0,0,TRUE,large_max_cliques);

	printf("Testing small: single 3-sized clique...");
	test_single_clique(small,3,0,FALSE,small_3_sized_cliques);

	printf("Testing large: single 8-sized clique...");
	test_single_clique(large,8,0,FALSE,large_8_sized_cliques);

	printf("\n");

	printf("Testing small: all maximum cliques...");
	test_all_cliques(small,0,0,FALSE,small_max_cliques);

	printf("Testing large: all maximum cliques...");
	test_all_cliques(large,0,0,TRUE,large_max_cliques);

	printf("Testing small: all min 3-sized cliques...");
	test_all_cliques(small,3,0,FALSE, small_3_sized_cliques);

	printf("Testing large: all min 8-sized cliques...");
	test_all_cliques(large,8,0,FALSE, large_8_sized_cliques);

	printf("Testing large: all exactly 8-sized cliques...");
	test_all_cliques(large,8,8,FALSE,large_exact_8_sized_cliques);

	printf("\n");

	printf("Testing large: max clique size...");
	test_max_weight(large,9);

	printf("Testing weighted large: max clique weight...");
	test_max_weight(wlarge,66);

	printf("\n");

	printf("Testing large: user_function w/ abort for 8-sized cliques...");
	test_user_function(large,8,large_8_sized_cliques);

	printf("Testing large: too small array for all maximum cliques...");
	test_too_small_array(large,8,large_8_sized_cliques);
	printf("\n");

	printf("Testing weighted large: single max weighted clique...");
	test_single_w_clique(wlarge,0,0,FALSE,large_w_max_cliques);

	printf("Testing weighted large: single min 60 weighted clique...");
	test_single_w_clique(wlarge,60,0,FALSE,large_w_60_sized_cliques);

	printf("Testing weighted large: single exactly 62 weighted clique...");
	test_single_w_clique(wlarge,62,62,FALSE,large_w_exact_62_cliques);

	printf("Testing weighted large: all max weighted cliques...");
	test_all_w_cliques(wlarge,0,0,TRUE,large_w_max_cliques);

	printf("Testing weighted large: all min 60 weighted cliques...");
	test_all_w_cliques(wlarge,60,0,FALSE,large_w_60_sized_cliques);

	printf("Testing weighted large: all 60...64 weighted maximal cliques...");
	test_all_w_cliques(wlarge,60,64,TRUE,large_w_60_64_maximal_cliques);
	printf("\n");

	printf("Testing re-entrance...");
	test_reentrance(wlarge,60,64,TRUE,large_w_60_64_maximal_cliques,
			small,0,0,FALSE,small_max_cliques);

	return 0;
}


static boolean list_contains(int (*sets)[N],set_t s,graph_t *g) {
	int i,j;
	boolean found;

	found=FALSE;
	for (i=0; sets[i][0]; i++) {
		if (set_size(s)==sets[i][0]) {
			found=TRUE;
			for (j=1; j<=sets[i][0]; j++) {
				if (!SET_CONTAINS(s,sets[i][j])) {
					found=FALSE;
					break;
				}
			}
		}
		if (found)
			break;
	}
	if (!found) {
		printf("ERROR (returned size=%d,",set_size(s));
		if (g)
			printf(" weight=%d,",graph_subgraph_weight(g,s));
		for (i=0; i<SET_MAX_SIZE(s); i++) {
			if (SET_CONTAINS(s,i))
				printf(" %d",i);
		}
		printf(")\n");
	}
	return found;
}


void test_single_clique(graph_t *g, int min_size, int max_size,
			boolean maximal, int (*sets)[N]) {
	set_t s;

	s=clique_unweighted_find_single(g,min_size,max_size,maximal,NULL);
	if (s==NULL) {
		printf("ERROR (returned NULL)\n");
		return;
	}

	if (!list_contains(sets,s,NULL))
		return;
	printf("OK\n");
	return;
}

void test_single_w_clique(graph_t *g,int min_size,int max_size,
			  boolean maximal, int (*sets)[N]){
	set_t s;
	int w;

	s=clique_find_single(g, min_size, max_size, maximal, NULL);
	if (s==NULL) {
		printf("ERROR (returned NULL)\n");
		return;
	}

	if (!list_contains(sets,s,g))
		return;
	w=graph_subgraph_weight(g,s);
	if (((min_size>0) && (w<min_size)) || ((max_size>0) && (w>max_size))) {
		printf("ERROR (wrong weight, w=%d)\n",w);
		return;
	}
	printf("OK (w=%d)\n",w);
	return;
}


void test_all_cliques(graph_t *g, int min_size, int max_size,
		      boolean maximal, int (*sets)[N]) {
	int correct_n;
	set_t s[1024];
	int n;
	int i;

	for (correct_n=0; sets[correct_n][0]; correct_n++)
		;
	clique_default_options->clique_list=s;
	clique_default_options->clique_list_length=1024;
	n=clique_unweighted_find_all(g,min_size,max_size,maximal,NULL);
	clique_default_options->clique_list=NULL;

	if (n!=correct_n) {
		printf("ERROR (returned %d cliques (!=%d))\n",n,correct_n);
		return;
	}

	for (i=0; i<n; i++) {
		if (!list_contains(sets,s[i],NULL))
			return;
	}
	printf("OK\n");
	return;
}

void test_all_w_cliques(graph_t *g,int min_size,int max_size,
			boolean maximal, int (*sets)[N]) {
	int correct_n;
	set_t s[1024];
	int n;
	int i;

	for (correct_n=0; sets[correct_n][0]; correct_n++)
		;
	clique_default_options->clique_list=s;
	clique_default_options->clique_list_length=1024;
	n=clique_find_all(g,min_size,max_size,maximal,NULL);
	clique_default_options->clique_list=NULL;

	if (n!=correct_n) {
		printf("ERROR (returned %d cliques (!=%d))\n",n,correct_n);
		return;
	}

	for (i=0; i<n; i++) {
		if (!list_contains(sets,s[i],g))
			return;
	}
	printf("OK\n");
	return;
}

static int user_fnct_cnt=0;
static boolean user_function_count(set_t s, graph_t *g, clique_options *opts) {
	int (*sets)[N] = opts->user_data;

	if (!list_contains(sets,s,g))
		return FALSE;
	user_fnct_cnt--;
	if (!user_fnct_cnt)
		return FALSE;
	return TRUE;
}


#define MAGIC 386
void test_user_function(graph_t *g, int min_size, int (*sets)[N]) {
	int n;

	user_fnct_cnt=MAGIC;
	clique_default_options->user_function=user_function_count;
	clique_default_options->user_data=sets;
	n=clique_unweighted_find_all(g,min_size,0,FALSE,
				     clique_default_options);
	clique_default_options->user_function=NULL;
	clique_default_options->user_data=NULL;

	if (n!=MAGIC) {
		printf("ERROR (returned %d cliques (!=%d))\n",n,MAGIC);
		return;
	}
	printf("OK\n");
	return;
}

#define MAGIC2 123
void test_too_small_array(graph_t *g,int min_size, int (*sets)[N]) {
	set_t s[MAGIC2+10];
	int i,n;
	int correct_n;

	for (correct_n=0; sets[correct_n][0]; correct_n++)
		;

	for (i=0; i<MAGIC2+10; i++)
		s[i]=NULL;

	clique_default_options->user_function=NULL;
	clique_default_options->clique_list=s;
	clique_default_options->clique_list_length=MAGIC2;
	
	n=clique_unweighted_find_all(g,min_size,0,FALSE,
				     clique_default_options);

	clique_default_options->user_data=NULL;
	clique_default_options->clique_list=NULL;

	for (i=MIN(n,MAGIC2); i<MAGIC2+10; i++) {
		if (s[i]!=NULL) {
			printf("ERROR (wrote too many cliques)\n");
			return;
		}
	}
	if (n!=correct_n) {
		printf("ERROR (found %d cliques (!=%d))\n",n,correct_n);
		return;
	}
	for (i=0; i<MIN(n,MAGIC2); i++) {
		if (!list_contains(sets,s[i],NULL))
			return;
	}
	printf("OK\n");
	return;
}


void test_max_weight(graph_t *g,int size) {
	int n;

	n=clique_max_weight(g,NULL);
	if (n!=size) {
		printf("ERROR (returned %d (!=%d))\n",n,size);
		return;
	}
	printf("OK\n");
	return;
}



struct searchopts {
	graph_t *g;
	int min,max;
	boolean maximal;
	int (*sets)[N];
};
boolean reentrance_user_function(set_t set, graph_t *graph,
				 clique_options *opts) {

	struct searchopts *sopt = (struct searchopts *)opts->user_data;
	clique_options localopts;
	int correct_n;
	set_t s[1024];
	int n;
	int i;

	for (correct_n=0; sopt->sets[correct_n][0]; correct_n++)
		;
	localopts.reorder_function=NULL;
	localopts.reorder_map=NULL;
	localopts.time_function=NULL;
	localopts.user_function=NULL;
	localopts.clique_list=s;
	localopts.clique_list_length=1024;
	n=clique_find_all(sopt->g,sopt->min,sopt->max,sopt->maximal,
			  &localopts);

	if (n!=correct_n) {
		printf("ERROR (returned %d cliques (!=%d))\n",n,correct_n);
		return FALSE;
	}

	for (i=0; i<n; i++) {
		if (!list_contains(sopt->sets,s[i],sopt->g)) {
			printf("(inner loop)\n");
			return FALSE;
		}
	}
	return TRUE;
}


void test_reentrance(graph_t *g1,int min1, int max1, boolean maximal1,
		     int (*sets1)[N], graph_t *g2,int min2,int max2,
		     boolean maximal2,int (*sets2)[N]) {

	struct searchopts sopt;
	int correct_n;
	set_t s[1024];
	int n;
	int i;

	for (correct_n=0; sets1[correct_n][0]; correct_n++)
		;

	sopt.g=g2;
	sopt.min=min2;
	sopt.max=max2;
	sopt.maximal=maximal2;
	sopt.sets=sets2;

	clique_default_options->user_function=reentrance_user_function;
	clique_default_options->user_data=&sopt;
	clique_default_options->clique_list=s;
	clique_default_options->clique_list_length=1024;
	n=clique_find_all(g1,min1,max1,maximal1,NULL);
	clique_default_options->user_function=NULL;
	clique_default_options->clique_list=NULL;

	if (n!=correct_n) {
		printf("ERROR (returned %d cliques (!=%d)) (outer loop)\n",
		       n,correct_n);
		return;
	}

	for (i=0; i<n; i++) {
		if (!list_contains(sets1,s[i],g1)) {
			printf("(outer loop)\n");
			return;
		}
	}
	printf("OK\n");
	return;
}


