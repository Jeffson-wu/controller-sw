#include <stdio.h>
#include <stdlib.h>
//#include <curl/curl.h>
#include "jsmn.h"

#include "json.h"
#include "log.h"
//#include "buf.h"
#include "gdi.h"

char URL[] = "https://api.twitter.com/1/trends/1.json";

char out[500]; /*buffer for debug printf*/
char out3[500]; /*buffer for debug printf*/
//#define log_die(fmt, args...)   

    typedef enum {
        START,
        WRAPPER, OBJECT,
        COMPONENTS, ARRAY,
        COMPONENT, NAME,
        DURATION, TEMPERATURE,
        CYCLES,CYCLES_WRAPPER,
        CYCLES_OBJECT,TUBES,
        TUBE,SKIP,
        STOP
    } parse_state;

const char * parse_st[] = {
	        "START",
        "WRAPPER", "OBJECT",
        "COMPONENTS", "ARRAY",
        "COMPONENT", "NAME",
        "DURATION", "TEMPERATURE",
        "CYCLES","CYCLES_WRAPPER",
        "CYCLES_OBJECT","TUBES",
        "TUBE","SKIP",
        "STOP"};


// stack is the state we return to when reaching the end of an object
parse_state stack[] = {STOP,STOP,STOP,STOP,STOP,STOP};
int stack_id = 0;


#define log_die(fmt, args...)      {sprintf(out, fmt, ## args);  gdi_send_msg_response(out);}
#define result(fmt, args...)      {sprintf(out, fmt, ## args);  gdi_send_msg_response(out);}

void push(parse_state object)
{
	//log_die("PUSH[%s]-%d",parse_st[stack[stack_id]],stack_id);
	stack[0]=object;
	//log_die("PUSH[%s]-[%s]-[%s]-[%s]-[%s]",parse_st[stack[0]],parse_st[stack[1]],parse_st[stack[2]],parse_st[stack[3]],parse_st[stack[4]]);
}

parse_state pull(void)
{
	//log_die("PULL[%s]-%d",parse_st[stack[stack_id-1]],stack_id-1);
	//log_die("PULL[%s]-[%s]-[%s]-[%s]-[%s]",parse_st[stack[0]],parse_st[stack[1]],parse_st[stack[2]],parse_st[stack[3]],parse_st[stack[4]]);

	return stack[0];
}


int json_test(void)
{
    //char *js = json_fetch(URL);
    
//	char js[] = "[ {\"pcrName\" : \"#Reverse transcription and PCR\",\"scriptName\" : \"Peters script 1\",\"user\" : \"Peter\",\"tubeConfig\" : [ 0, 3, 15 ],\"finishedAt\" : \"Nov 7, 2013 3:05:38 PM\",\"elapsedTime\" : 960,	\"minicube\" : {		\"name\" : \"Minicube 1\",		\"numTubes\" : 16,		\"color\" : \"#928bb7\" },\"cycles\" :[ {		\"components\" : [ {			\"duration\":123,			\"temperature\" : 60		}]}]}]";						

//	char js[] = "[ {\"cycles\" :[ {		\"components\" : [ {			\"duration\":123,			\"temperature\" : 60		}]}]}]";
//	char js[] = "[ {		\"components\" : [ {			\"duration\" : 128,			\"temperature\" : 60		} ] },{		\"components\" : [ {			\"duration\" : 130,			\"temperature\" : 62		} ] } ]";
	//char js[] = "[ {\"components\" : [ {\"duration\" : 128,\"temperature\" : 60} ] },{\"components\" : [ {\"duration\" : 130,\"temperature\" : 62} ] },{\"components\" : [ {\"duration\" : 140,\"temperature\" : 70} ] },{\"components\" : [ {\"duration\" : 150,\"temperature\" : 80} ] },{\"components\" : [ {\"duration\" : 160,\"temperature\" : 90} ] } ]";
// char js[] ="[ {	\"pcrName\" : \"#Reverse transcription and PCR\",	\"scriptName\" : \"Peters script 1\",	\"user\" : \"Peter\",	\"tubeConfig\" : [ 0, 3, 15 ],	\"finishedAt\" : \"Nov 7, 2013 3:05:38 PM\",	\"elapsedTime\" : 960,	\"minicube\" : {		\"name\" : \"Minicube 1\",		\"numTubes\" : 16,		\"color\" : \"#928bb7\"	},	\"cycles\" : [ {		\"components\" : [ {			\"duration\" : 120,			\"temperature\" : 60		} ]	}, {		\"components\" : [ {			\"duration\" : 1,			\"temperature\" : 95		} ]	}]}]";
char js[] ="[ {	\"pcrName\" : \"#Reverse transcription and PCR\",	\"scriptName\" : \"Peters script 1\",	\"user\" : \"Peter\",	\"tubeConfig\" : [ 0, 3, 15 ],	\"finishedAt\" : \"Nov 7, 2013 3:05:38 PM\",	\"elapsedTime\" : 960,	\"minicube\" : {		\"name\" : \"Minicube 1\",		\"numTubes\" : 16,		\"color\" : \"#928bb7\" },	\"cycles\" : [ {		\"components\" : [ {			\"duration\" : 120, 		\"temperature\" : 60		},{			\"duration\" : 120, 		\"temperature\" : 60		} ] }, {		\"components\" : [ {			\"duration\" : 1,			\"temperature\" : 95		} ] }, {		\"components\" : [ {			\"duration\" : 1,			\"temperature\" : 95		} ] }, {		\"components\" : [ {			\"duration\" : 1,			\"temperature\" : 95		} ] }, {		\"components\" : [ {			\"duration\" : 1,			\"temperature\" : 95		} ] }]}]";

//char js[] = "[ {	\"pcrName\" : \"#Reverse transcription and PCR\",	\"scriptName\" : \"Peters script 1\",	\"user\" : \"Peter\",	\"tubeConfig\" : [ 0, 3, 15 ],	\"finishedAt\" : \"Nov 7, 2013 3:05:38 PM\",	\"elapsedTime\" : 960,	\"minicube\" : {		\"name\" : \"Minicube 1\",		\"numTubes\" : 16,		\"color\" : \"#928bb7\"	},	\"cycles\" : [ {		\"components\" : [ {			\"duration\" : 120,			\"temperature\" : 60		} ]	}, {		\"components\" : [ {			\"duration\" : 1,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 55		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	} ]}, {	\"scriptName\" : \"Peters script 2\",	\"pcrName\" : \"Nested PCR, add primers when pausing\",	\"user\" : \"Peter\",	\"tubeConfig\" : [ 4, 1, 2 ],	\"finishedAt\" : null,	\"elapsedTime\" : 17,	\"minicube\" : {		\"name\" : \"Minicube 1\",		\"numTubes\" : 16,		\"color\" : \"#928bb7\"	},	\"cycles\" : [ {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 57		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	}, {		\"components\" : [ {			\"duration\" : 5,			\"temperature\" : 57		}, {			\"duration\" : 10,			\"temperature\" : 72		}, {			\"duration\" : 5,			\"temperature\" : 95		} ]	} ]} ]";
    jsmntok_t *tokens = json_tokenise(js);

    /* The Twitter trends API response is in this format:
     *
     * [
     *   {
     *      ...,
     *      "trends": [
     *          {
     *              ...,
     *              "name": "TwitterChillers",
     *          },
     *          ...,
     *      ],
     *      ...,
     *   }
     * ]
     *
     */

//  const char* jsmn_type[]= {
//    "JSMN_PRIMITIVE",
//    "JSMN_OBJECT",
//    "JSMN_ARRAY",
//    "JSMN_STRING"
//  };


    // state is the current state of the parser
    parse_state state = START;

    // stack is the state we return to when reaching the end of an object
  //  parse_state stack[] = {STOP,STOP};
  //  int stack_id = 0;
    // Counters to keep track of how far through parsing we are
    size_t wrapper_tokens = 0;
	size_t cycles_wrapper_tokens = 0;
	size_t cycles_object_tokens = 0;
    size_t object_tokens = 0;
    size_t skip_tokens = 0;
    size_t components = 0;
    size_t tubes = 0;
    size_t tube_temp_time = 0;
	size_t i,j;

    for ( i = 0, j = 1; j > 0; i++, j--)
    {
        jsmntok_t *t = &tokens[i];

        // Should never reach uninitialized tokens
        log_assert(t->start != -1 && t->end != -1);
      //  memset(out3,60,0);
        if (t->type == JSMN_ARRAY || t->type == JSMN_OBJECT)
            j += t->size;
	//	strncpy(out3, js + tokens[i].start, ((tokens[i].end - tokens[i].start)<20)?(tokens[i].end - tokens[i].start):20);
	//	out3[(tokens[i].end - tokens[i].start)]='\0';
		//log_die("%s[%d]-%s",jsmn_type[tokens[i].type],j,out3);

   // 	}
#if 1
		//log_die("STATE:[%s-%s]-%d CNT[%d:%d] START[%d] END[%d] SIZE[%d] TYPE[%s] [%s]",parse_st[state],parse_st[stack[0]],stack_id,i,j,tokens[i].start,tokens[i].end,tokens[i].size,jsmn_type[tokens[i].type],out3);
		//json_token_streq(js, t, "trends");
		

        switch (state)
        {
			

            case START:
                if (t->type != JSMN_ARRAY)
                    log_die("Invalid response: root element must be array.");
                if (t->size != 1)
                    log_die("Invalid response: array must have one element.");

                state = WRAPPER;
				wrapper_tokens = t->size;
                break;

            case WRAPPER:
				wrapper_tokens--;
                if (t->type != JSMN_OBJECT)
                    log_die("Invalid response: wrapper must be an object.");

                state = OBJECT;
                object_tokens = t->size;
                break;

            case OBJECT:
                object_tokens--;
				//log_die("OBJECT [%d] %d",object_tokens,state);

                // Keys are odd-numbered tokens within the object
                if (object_tokens % 2 == 1)
                {
					//log_die("OBJECT [%s]",js);
                //	if (t->type == JSMN_STRING && json_token_streq(js, t, "components"))
                //        state = COMPONENTS;
					if (t->type == JSMN_STRING && json_token_streq(js, t, "cycles"))
                        state = CYCLES;
                    if (t->type == JSMN_STRING && json_token_streq(js, t, "tubeConfig"))
                    {
                        state = TUBES;
                        push(OBJECT);
                    }
                }
                else if (t->type == JSMN_ARRAY || t->type == JSMN_OBJECT)
                {
                    state = SKIP;
                    push(OBJECT);
					
                    skip_tokens = t->size;
                }

                // Last object value
                if (object_tokens == 0)
                    state = STOP;
				

                break;

            case SKIP:
                skip_tokens--;

                if (t->type == JSMN_ARRAY || t->type == JSMN_OBJECT)
                    skip_tokens += t->size;

                if (skip_tokens == 0)
                    state = pull();

                break;

         
			case TUBES:
				object_tokens--; /*Take 1 object of que since Tubes array is also counted as an object*/
						  if (t->type != JSMN_ARRAY)
				  log_die("Unknown components value: expected array.");
	
				  tubes = t->size;
				  state = TUBE;

	              if(tubes == 0)
	            	  state=pull();
				  // No tubes found
				 
	
				  break;
            case TUBE:
			     tubes--;
     			if (t->type == JSMN_PRIMITIVE)
     			{
     			  char *str = json_token_tostr(js, t);
     			  result("FOUND TUBE: %d",atoi(str));
				  // Last tube value
                  if (tubes == 0)
   				      state = pull();
     			}  
            break;
            case ARRAY:
				 if (t->type != JSMN_OBJECT)
                    log_die("Unknown components value: expected object.");
                components--;

                tube_temp_time = t->size;
                state = COMPONENT;

                // Empty trend object
                if (tube_temp_time == 0)
                    state = STOP;

                // Last trend object
                if (components == 0)
                    push(STOP);

                break;
				case CYCLES:
					if (t->type != JSMN_ARRAY)
						log_die("Invalid response: cycles element must be array ");
					if (t->size < 1)
						log_die("Invalid response: array must have at least one element.");
				
					state = CYCLES_WRAPPER;
					cycles_wrapper_tokens = t->size;
					result("FOUND %d CYCLES",cycles_wrapper_tokens);
					break;
				
				case CYCLES_WRAPPER:
					cycles_wrapper_tokens--;
					if (t->type != JSMN_OBJECT)
						log_die("Invalid response: cycles wrapper must be an object.");
				
					state = CYCLES_OBJECT;
					cycles_object_tokens = t->size;
					break;
				
				case CYCLES_OBJECT:
					cycles_object_tokens--;
					//log_die("OBJECT [%d] %d",object_tokens,state);
				
					// Keys are odd-numbered tokens within the object
					if (object_tokens % 2 == 1)
					{
						//log_die("OBJECT [%s]",js);
						if (t->type == JSMN_STRING && json_token_streq(js, t, "components"))
							  state = COMPONENTS;
					
					}
					else if (t->type == JSMN_ARRAY || t->type == JSMN_OBJECT)
					{
						state = SKIP;
						push(OBJECT);
						
						skip_tokens = t->size;
					}
				
					// Last object value
					if (object_tokens == 0)
						state = STOP;
					
				
					break;
				case COMPONENTS:
							 if (t->type != JSMN_ARRAY)
								 log_die("Unknown components value: expected array.");
				
							 components = t->size;
							result("FOUND %d COMPONENTS",components);
							 state = ARRAY;
							 push(ARRAY);
				
							 // No components found
							 if (components == 0)
								 state = STOP;
				
						 break;
            case COMPONENT:
            case NAME:
                tube_temp_time--;

                // Keys are odd-numbered tokens within the object
                if (tube_temp_time % 2 == 1)
                {
					//log_die("tube_temp_time");
                    if (t->type == JSMN_STRING && json_token_streq(js, t, "duration"))
                        state = DURATION;
					
                    if (t->type == JSMN_STRING && json_token_streq(js, t, "temperature"))
                        state = TEMPERATURE;
                }
                else if (t->type == JSMN_ARRAY || t->type == JSMN_OBJECT)
                {
                    state = SKIP;
                    push(COMPONENT);
                    skip_tokens = t->size;
                }

                // Last object value
                if (tube_temp_time == 0)
                    state = pull();

                break;
			case DURATION:
 				    tube_temp_time--;
					if (t->type == JSMN_PRIMITIVE)
					{
					  char *str = json_token_tostr(js, t);
					  result("FOUND DURATION: %d",atoi(str));
					}
					state = COMPONENT;
					// Last object value
                if (tube_temp_time == 0)
                {
					 // Last object value
                  if (cycles_wrapper_tokens > 0)
                  {
                    state = CYCLES_WRAPPER;
                  }else
                  {
    				  if (wrapper_tokens==0)
    				   {
    					  result("END OF JSON");
    				   }
					state = pull();
                  }
				  if (components > 0)
					state = ARRAY;
                }
				
//                    state = stack;
				break;
			case TEMPERATURE:
				    tube_temp_time--;
					if (t->type == JSMN_PRIMITIVE)
					{
					  char *str = json_token_tostr(js, t);
					  result("FOUND TEMPERATURE: %d",atoi(str));
					}
					state = COMPONENT;
						// Last object value
					if (tube_temp_time == 0)
					{
						 // Last object value
					  if (cycles_wrapper_tokens > 0)
					  {
						state = CYCLES_WRAPPER;
					  }else
					  {
                       if (wrapper_tokens==0)
                       	{
					       result("END OF JSON");
                       	}
						state = pull();
					  }
					  if (components > 0)
					state = ARRAY;
					}

				

				break;

            case STOP:
                // Just consume the tokens
                break;

            default:
                log_die("Invalid state %u", state);
        }
    }

    return 0;
#endif

}

/*

Rec:(14:21:58)

FOUND TUBE: 0

FOUND TUBE: 3

FOUND TUBE: 15

FOUND 5 CYCLES

FOUND 2 COMPONENTS

FOUND DURATION: 120

FOUND TEMPERATURE: 60

FOUND DURATION: 120

FOUND TEMPERATURE: 60

FOUND 1 COMPONENTS

FOUND DURATION: 1

FOUND TEMPERATURE: 95

FOUND 1 COMPONENTS

FOUND DURATION: 1

FOUND TEMPERATURE: 95

FOUND 1 COMPONENTS

FOUND DURATION: 1

FOUND TEMPERATURE: 95

FOUND 1 COMPONENTS

FOUND DURATION: 1

FOUND TEMPERATURE: 95

END OF JSON


*/
