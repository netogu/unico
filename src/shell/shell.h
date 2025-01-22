#ifndef SHELL_H
#define SHELL_H

#include <microshell.h>
#include <string.h>
#include <ush_config.h>

// stringify helper - use xstr() to convert #define to a usable string
#define str(s) #s
#define xstr(s) str(s)
#define PRINT_FLOAT(x) printf("%d.%03d", (int)(x), (int)((x - (int)(x)) * 1000))

extern const char *shell_head;

void shell_init(void);
void shell_update(void);
char *timestamp(void);

#endif // SHELL_H
