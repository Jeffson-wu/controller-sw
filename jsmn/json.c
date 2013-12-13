//#include <curl/curl.h>
#include <stdlib.h>

#include "json.h"
//#include "log.h"
//#include "buf.h"

#define BUFFER_SIZE 32768
#define JSON_TOKENS 256

char out1[300]; /*buffer for debug printf*/
char out2[60]; /*buffer for debug printf*/


#define log_die(fmt, args...)      sprintf(out1, fmt, ## args);  gdi_send_msg_response(out1);


#if 0

static size_t fetch_data(void *buffer, size_t size, size_t nmemb, void *userp)
{
    buf_t *buf = (buf_t *) userp;
    size_t total = size * nmemb;

    if (buf->limit - buf->len < total)
    {
        buf = buf_size(buf, buf->limit + total);
        log_null(buf);
    }

    buf_concat(buf, buffer, total);

    return total;
}

char * json_fetch(char *url)
{
    CURL *curl = curl_easy_init();
    log_null(curl);

    curl_easy_setopt(curl, CURLOPT_URL, url);

    buf_t *buf = buf_size(NULL, BUFFER_SIZE);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, fetch_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, buf);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "jsmn-example (https://github.com/alisdair/jsmn-example, alisdair@mcdiarmid.org)");

    struct curl_slist *hs = curl_slist_append(NULL, "Accept: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hs);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK)
        log_die("curl_easy_perform failed: %s", curl_easy_strerror(res));

    curl_easy_cleanup(curl);
    curl_slist_free_all(hs);

    char *js = buf_tostr(buf);
    free(buf);

    return js;
}
#endif
jsmntok_t * json_tokenise(char *js)
{
    jsmn_parser parser;
    jsmn_init(&parser);

    unsigned int n = JSON_TOKENS;
    jsmntok_t *tokens = (jsmntok_t*)pvPortMalloc(sizeof(jsmntok_t) * n);
    log_null(tokens);

    int ret = jsmn_parse(&parser, js, tokens, n);

    while (ret == JSMN_ERROR_NOMEM)
    {
        n = n * 2 + 1;
        tokens = realloc(tokens, sizeof(jsmntok_t) * n);
        log_null(tokens);
        ret = jsmn_parse(&parser, js, tokens, n);
    }

    if (ret == JSMN_ERROR_INVAL)
        log_die("jsmn_parse: invalid JSON string");
    if (ret == JSMN_ERROR_PART)
        log_die("jsmn_parse: truncated JSON string");

    return tokens;
}

bool json_token_streq(char *js, jsmntok_t *t, char *s)
{
#if 0
	strncpy(out2, js + t->start, t->end - t->start);
	out2[t->end - t->start]='\0';
	log_die("MATCH TOKEN: %s = %s",out2,s);
#endif

	return (strncmp(js + t->start, s, t->end - t->start) == 0
            && strlen(s) == (size_t) (t->end - t->start));
}

char * json_token_tostr(char *js, jsmntok_t *t)
{
    js[t->end] = '\0';
    return js + t->start;
}
