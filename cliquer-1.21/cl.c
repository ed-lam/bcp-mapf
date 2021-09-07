
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef ENABLE_LONG_OPTIONS
#include <getopt.h>
#endif

#include "cliquer.h"


#define TRYFORHELP  "Try `%s -h' for more information.\n",argv[0]

void printhelp(char *prog);
void read_options(int argc, char **argv);
void print_search(graph_t *g);
boolean record_clique_func(set_t s,graph_t *g,clique_options *opts);
boolean print_clique_func(set_t s,graph_t *g,clique_options *opts);
void print_clique(set_t s,graph_t *g);


/* Options, changed by command-line arguments. */
static boolean find_all=FALSE;
static int min_weight=0;
static boolean min_weight_set=FALSE;
static int max_weight=0;
static boolean max_weight_set=FALSE;
static boolean maximal=FALSE;
static boolean unweighted=FALSE;
static boolean number1=TRUE;
static int quiet=0;
static boolean only_weight=FALSE;
static int *(*reorder)(graph_t *, boolean)=reorder_by_default;
static char *file;

/* Dynamically allocated storage for cliques. */
static set_t *clique_list;
static int clique_count=0;
static int clique_list_size=0;


int main(int argc, char **argv) {
	FILE *fp;
	graph_t *g;
	set_t s;
	clique_options *opts;
	int i,j;

	if (argc<=1)
		printhelp(argv[0]);
	read_options(argc,argv);

	if (file[0]=='-' && file[1]==0) {
		fp=stdin;
		if (quiet<=1)
			fprintf(stderr,"Reading graph from stdin...");
	} else {
		fp=fopen(file,"rb");
		if (fp==NULL) {
			perror(file);
			exit(2);
		}
		if (quiet<=1)
			fprintf(stderr,"Reading graph from %s...",file);
	}
	g=graph_read_dimacs(fp);
	if (g==NULL) {
		fprintf(stderr,"Error in graph file.\n");
		return 1;
	}
	if (quiet<=1)
		fprintf(stderr,"OK\n");
	fclose(fp);

	/* Set stdout to be line-buffered even if redirected. */
	setvbuf(stdout,(char *)NULL,_IOLBF,0);

	/* Initialize out clique_options */
	opts=malloc(sizeof(clique_options));
	if (quiet)
		opts->time_function=NULL;
	else
		opts->time_function=clique_print_time;
	opts->output=stderr;
	opts->reorder_function=reorder;
	opts->reorder_map=NULL;
	if (quiet)
		opts->user_function=print_clique_func;
	else
		opts->user_function=record_clique_func;
	opts->user_data=NULL;
	opts->clique_list=NULL;
	opts->clique_list_length=0;

	/* Report what we are doing. */
	if (quiet<=1)
		print_search(g);

	if (only_weight) {
		if (unweighted) {
			j=clique_unweighted_max_weight(g,opts);
			printf("Largest clique: %d\n",j);
		} else {
			j=clique_max_weight(g,opts);
			printf("Heaviest clique: %d\n",j);
		}
	} else if (find_all) {
		if (unweighted) {
			clique_unweighted_find_all(g,min_weight,max_weight,
						   maximal,opts);
		} else {
			clique_find_all(g,min_weight,max_weight,maximal,opts);
		}
		if (quiet<=0) {
			fprintf(stderr,"Found %d clique%s:\n",
				clique_count,(clique_count==1)?"":"s");
			for (i=0; i<clique_count; i++) {
				print_clique(clique_list[i],g);
			}
		}
	} else {
		if (unweighted) {
			s=clique_unweighted_find_single(g,min_weight,
							max_weight,maximal,
							opts);
		} else {
			s=clique_find_single(g,min_weight,max_weight,maximal,
					     opts);
		}
		if (s==NULL) {
			if (quiet<=1)
				fprintf(stderr,"No such clique found.\n");
			return 0;
		}
		print_clique(s,g);
	}

	return 0;
}


/*
 * Prints the help message to stdout.  prog is the program name
 * (from argv[0]).
 */
void printhelp(char *prog) {
	printf("Usage:  %s [options] <dimacs-file>\n"
	       "\n"
	       "<dimacs-file> is the ASCII or binary DIMACS-format file from which to read\n"
	       "the graph.  Options are the following:\n"
	       "\n"
	       " -h    --help         This help text.\n"
	       " -a    --all          Find all cliques.\n"
	       " -s    --single       Find only one clique (default).\n"
	       " -w    --weight       Tell only maximum weight (no faster than -s).\n"
	       " -m N  --min N        Search for cliques with weight at least N.  If N=0,\n"
	       "                      searches for maximum weight clique (default).\n"
	       " -M N  --max N        Search for cliques with weight at most N.  If N=0,\n"
	       "                      no limit is imposed (default).  N being positive is\n"
	       "                      incompatible with \"--min 0\" (\"--min 1\" is assumed).\n"
	       " -x    --maximal      Require cliques to be maximal.\n"
	       " -u    --unweighted   Assume weight 1 for all vertices.\n"
	       " -0    --from-0       Number vertices 0 to n-1 instead of 1 to n when writing.\n"
	       " -r F  --reorder F    Reorder with function F.  See below for details.\n"
	       " -q    --quiet        Suppresses progress output.  Specifying -q twice\n"
	       "                      suppresses all output except the actual result.\n"
	       "\n"
	       "Available reordering functions are the following:\n"
	       "\n"
	       "      none        No ordering (same order as in the file).\n"
	       "      reverse     Reverse order as in the file.\n"
	       "      default     One of the two below, depending on weightedness.\n"
	       "      unweighted-coloring   Coloring method efficient for unweighted graphs.\n"
	       "      weighted-coloring     Coloring method efficient for weighted graphs.\n"
	       "      degree      Order by ascending degree.\n"
	       "      random      Random order.\n"
	       "\n",
	       prog);
#ifndef ENABLE_LONG_OPTIONS
	printf("(Long options not enabled during compile.)\n\n");
#endif
	exit(0);
}


/*
 * Reads the command-line options and sets the global options accordingly.
 */
void read_options(int argc, char **argv) {
	int c;
	char *ptr;

	while (TRUE) {
#ifdef ENABLE_LONG_OPTIONS
		int option_index = 0;
		static struct option long_options[] = {
			{ "all", no_argument, NULL, 'a' },
			{ "single", no_argument, NULL, 's' },
			{ "weight", no_argument, NULL, 'w' },
			{ "min", required_argument, NULL, 'm' },
			{ "max", required_argument, NULL, 'M' },
			{ "maximal", no_argument, NULL, 'x' },
			{ "unweighted", no_argument, NULL, 'u' },
			{ "reorder", required_argument, NULL, 'r' },
			{ "from-0", no_argument, NULL, '0' },
			{ "quiet", no_argument, NULL, 'q' },
			{ "help", no_argument, NULL, 'h' },
			{ 0,0,0,0 }
		};

		c=getopt_long(argc,argv,"aswm:M:xur:1qh",
			      long_options,&option_index);
#else  /* !ENABLE_LONG_OPTIONS */
		c=getopt(argc,argv,"aswm:M:xur:1qh-");
#endif /* !ENABLE_LONG_OPTIONS */
		if (c==-1)
			break;

		switch (c) {
		case 0:
			break;
		case 'a':
			find_all=TRUE;
			break;
		case 's':
			find_all=FALSE;
			break;
		case 'w':
			only_weight=TRUE;
			break;
		case 'm':
			min_weight=strtol(optarg,&ptr,10);
			if ((*optarg == 0) || (*ptr != 0) || (min_weight<0)) {
				fprintf(stderr,"Bad argument: %s\n",optarg);
				fprintf(stderr,TRYFORHELP);
				exit(1);
			}
			if (min_weight==0 && max_weight>0) {
				if (max_weight_set) {
					fprintf(stderr,"Error: --max >0 is "
						"incompatible with --min 0\n");
					fprintf(stderr,TRYFORHELP);
					exit(1);
				}
				max_weight=0;
			}
			min_weight_set=TRUE;
			break;
		case 'M':
			max_weight=strtol(optarg,&ptr,10);
			if ((*optarg == 0) || (*ptr != 0) || (max_weight<0)) {
				fprintf(stderr,"Bad argument: %s\n",optarg);
				fprintf(stderr,TRYFORHELP);
				exit(1);
			}
			if (max_weight>0 && min_weight==0) {
				if (min_weight_set) {
					fprintf(stderr,"Error: --max >0 is "
						"incompatible with --min 0\n");
					fprintf(stderr,TRYFORHELP);
					exit(1);
				}
				min_weight=1;
			}
			max_weight_set=TRUE;
			break;
		case 'x':
			maximal=TRUE;
			break;
		case 'u':
			unweighted=TRUE;
			break;
		case 'r':
			if (strcasecmp(optarg,"none")==0)
				reorder=NULL;
			else if (strcasecmp(optarg,"reverse")==0)
				reorder=reorder_by_reverse;
			else if (strcasecmp(optarg,"default")==0)
				reorder=reorder_by_default;
			else if (strcasecmp(optarg,"unweighted-coloring")==0)
				reorder=reorder_by_unweighted_greedy_coloring;
			else if (strcasecmp(optarg,"weighted-coloring")==0)
				reorder=reorder_by_weighted_greedy_coloring;
			else if (strcasecmp(optarg,"degree")==0)
				reorder=reorder_by_degree;
			else if (strcasecmp(optarg,"random")==0)
				reorder=reorder_by_random;
			else {
				fprintf(stderr,"Bad reordering type: %s\n",
					optarg);
				fprintf(stderr,TRYFORHELP);
				exit(1);
			}
			break;
		case '0':
			number1=FALSE;
			break;
		case 'q':
			quiet++;
			break;
		case 'h':
			printhelp(argv[0]);
			break;
#ifndef ENABLE_LONG_OPTIONS
		case '-':
			fprintf(stderr,"%s: Long options not enabled "
				"during compile.\n",argv[0]);
			exit(1);
#endif
		case '?':
			fprintf(stderr,TRYFORHELP);
			exit(1);
		default:
			fprintf(stderr,"Internal bug: line %d should never "
				"be reached!",__LINE__);
			exit(-1);
		}
	}

	if (optind >= argc) {
		fprintf(stderr,"Missing filename.\n");
		fprintf(stderr,TRYFORHELP);
		exit(1);
	}
	if (optind < argc-1) {
		fprintf(stderr,"Bad command line option: %s\n",argv[optind+1]);
		fprintf(stderr,TRYFORHELP);
		exit(1);
	}

	file=argv[optind];
	return;
}


/*
 * Prints a line to stderr indicating what is being searched for.
 */
void print_search(graph_t *g) {
	fprintf(stderr,"Searching for %s ",find_all?"all":"a single");
	if (min_weight==0) {
		fprintf(stderr,"maximum %s clique%s...\n",
			unweighted?"size":"weight",find_all?"s":"");
		return;
	}
	if (min_weight>0 && maximal)
		fprintf(stderr,"maximal ");
	fprintf(stderr,"clique%s with %s ",find_all?"s":"",
		unweighted?"size":"weight");
	if (max_weight==0) {
		fprintf(stderr,"at least %d...",min_weight);
	} else if (min_weight==max_weight) {
		fprintf(stderr,"exactly %d...",min_weight);
	} else {
		fprintf(stderr,"within range %d-%d...",min_weight,max_weight);
	}
	fprintf(stderr,"\n");
	return;
}

/*
 * Prints the clique s along with it's size and weight.
 */
void print_clique(set_t s,graph_t *g) {
	int i;

	printf("size=%d, weight=%d:  ",set_size(s),graph_subgraph_weight(g,s));
	for (i=0; i<SET_MAX_SIZE(s); i++) {
		if (SET_CONTAINS(s,i)) {
			if (number1)
				printf(" %d",i+1);
			else
				printf(" %d",i);
		}
	}
	printf("\n");
	return;
}

/*
 * Records a clique into the clique list using dynamic allocation.
 * Used as opts->user_function.
 */
boolean record_clique_func(set_t s,graph_t *g,clique_options *opts) {
	if (clique_count>=clique_list_size) {
		clique_list=realloc(clique_list,(clique_list_size+512) * 
				    sizeof(set_t));
		clique_list_size+=512;
	}
	clique_list[clique_count]=set_duplicate(s);
	clique_count++;
	return TRUE;
}

/*
 * Prints a clique.  Used as opts->user_function.
 */
boolean print_clique_func(set_t s,graph_t *g,clique_options *opts) {
	print_clique(s,g);
	return TRUE;
}

