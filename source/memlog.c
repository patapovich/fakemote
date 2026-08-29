#include <stdarg.h>
#include <string.h>
#include "types.h"
#include "utils.h"
#include "tools.h"

extern void MEM2_Prot(u32 flag);

/* Ring log at a fixed MEM2 location readable from the PPC side
 * (PPC 0x93200000). GX's MEM2 heap ends at 0x93000000 and IOS
 * private memory starts at 0x933E0000, so this range is quiet. */
#define MEMLOG_ADDR  0x13200000
#define MEMLOG_MAGIC 0x464d4c47 /* FMLG */
#define MEMLOG_SIZE  0x4000

struct memlog {
	u32 magic;
	u32 len;
	char data[MEMLOG_SIZE];
};

static struct memlog *const s_log = (struct memlog *)MEMLOG_ADDR;
static int s_initialized;

extern int vsnprintf(char *str, unsigned int size, const char *format, va_list ap);

void memlog(const char *fmt, ...)
{
	char line[128];
	va_list args;
	int n;

	if (!s_initialized) {
		MEM2_Prot(0); /* ensure PPC can see MEM2 writes */
		s_log->magic = MEMLOG_MAGIC;
		s_log->len = 0;
		s_initialized = 1;
	}

	va_start(args, fmt);
	n = vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	if (n <= 0)
		return;
	if (n > (int)sizeof(line))
		n = sizeof(line);

	if (s_log->len + n > MEMLOG_SIZE)
		s_log->len = 0; /* wrap by truncating */
	memcpy(&s_log->data[s_log->len], line, n);
	s_log->len += n;

	/* Flush so the PPC side sees it through uncached MEM2 */
	DCFlushRange(s_log, sizeof(*s_log));
}
