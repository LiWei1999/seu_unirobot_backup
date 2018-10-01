#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "option_list.h"
#include "utils.h"

clist *read_data_cfg(char *filename)
{
    FILE *file = fopen(filename, "r");

    if (file == 0)
    {
        file_error(filename);
    }

    char *line;
    int nu = 0;
    clist *options = make_list();

    while ((line = fgetl(file)) != 0)
    {
        ++ nu;
        strip(line);

        switch (line[0])
        {
            case '\0':
            case '#':
            case ';':
                free(line);
                break;

            default:
                if (!read_option(line, options))
                {
                    fprintf(stderr, "Config file error line %d, could parse: %s\n", nu, line);
                    free(line);
                }

                break;
        }
    }

    fclose(file);
    return options;
}

metadata get_metadata(char *file)
{
    metadata m = {0};
    clist *options = read_data_cfg(file);

    char *name_list = option_find_str(options, "names", 0);

    if (!name_list)
    {
        name_list = option_find_str(options, "labels", 0);
    }

    if (!name_list)
    {
        fprintf(stderr, "No names or labels found\n");
    }
    else
    {
        m.names = get_labels(name_list);
    }

    m.classes = option_find_int(options, "classes", 2);
    free_list(options);
    return m;
}

int read_option(char *s, clist *options)
{
    size_t i;
    size_t len = strlen(s);
    char *val = 0;

    for (i = 0; i < len; ++i)
    {
        if (s[i] == '=')
        {
            s[i] = '\0';
            val = s + i + 1;
            break;
        }
    }

    if (i == len - 1)
    {
        return 0;
    }

    char *key = s;
    option_insert(options, key, val);
    return 1;
}

void option_insert(clist *l, char *key, char *val)
{
    kvp *p = malloc(sizeof(kvp));
    p->key = key;
    p->val = val;
    p->used = 0;
    list_insert(l, p);
}

void option_unused(clist *l)
{
    node *n = l->front;

    while (n)
    {
        kvp *p = (kvp *)n->val;

        if (!p->used)
        {
            fprintf(stderr, "Unused field: '%s = %s'\n", p->key, p->val);
        }

        n = n->next;
    }
}

char *option_find(clist *l, char *key)
{
    node *n = l->front;

    while (n)
    {
        kvp *p = (kvp *)n->val;

        if (strcmp(p->key, key) == 0)
        {
            p->used = 1;
            return p->val;
        }

        n = n->next;
    }

    return 0;
}
char *option_find_str(clist *l, char *key, char *def)
{
    char *v = option_find(l, key);

    if (v)
    {
        return v;
    }

    if (def)
    {
        fprintf(stderr, "%s: Using default '%s'\n", key, def);
    }

    return def;
}

int option_find_int(clist *l, char *key, int def)
{
    char *v = option_find(l, key);

    if (v)
    {
        return atoi(v);
    }

    fprintf(stderr, "%s: Using default '%d'\n", key, def);
    return def;
}

int option_find_int_quiet(clist *l, char *key, int def)
{
    char *v = option_find(l, key);

    if (v)
    {
        return atoi(v);
    }

    return def;
}

float option_find_float_quiet(clist *l, char *key, float def)
{
    char *v = option_find(l, key);

    if (v)
    {
        return atof(v);
    }

    return def;
}

float option_find_float(clist *l, char *key, float def)
{
    char *v = option_find(l, key);

    if (v)
    {
        return atof(v);
    }

    fprintf(stderr, "%s: Using default '%lf'\n", key, def);
    return def;
}
