/*
 * ARM64 optimized string functions
 */

#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dest, const void *src, size_t n)
{
    uint8_t *d = dest;
    const uint8_t *s = src;
    
    /* Handle small copies byte by byte */
    if (n < 16) {
        while (n--)
            *d++ = *s++;
        return dest;
    }
    
    /* Align destination to 8 bytes */
    while ((uintptr_t)d & 7) {
        *d++ = *s++;
        n--;
    }
    
    /* Copy 8 bytes at a time */
    uint64_t *d8 = (uint64_t *)d;
    const uint64_t *s8 = (const uint64_t *)s;
    
    while (n >= 8) {
        *d8++ = *s8++;
        n -= 8;
    }
    
    /* Handle remaining bytes */
    d = (uint8_t *)d8;
    s = (const uint8_t *)s8;
    while (n--)
        *d++ = *s++;
    
    return dest;
}

void *memset(void *s, int c, size_t n)
{
    uint8_t *p = s;
    uint64_t val8 = (uint8_t)c;
    val8 |= val8 << 8;
    val8 |= val8 << 16;
    val8 |= val8 << 32;
    
    /* Handle small sets byte by byte */
    if (n < 16) {
        while (n--)
            *p++ = (uint8_t)c;
        return s;
    }
    
    /* Align to 8 bytes */
    while ((uintptr_t)p & 7) {
        *p++ = (uint8_t)c;
        n--;
    }
    
    /* Set 8 bytes at a time */
    uint64_t *p8 = (uint64_t *)p;
    while (n >= 8) {
        *p8++ = val8;
        n -= 8;
    }
    
    /* Handle remaining bytes */
    p = (uint8_t *)p8;
    while (n--)
        *p++ = (uint8_t)c;
    
    return s;
}

int memcmp(const void *s1, const void *s2, size_t n)
{
    const uint8_t *p1 = s1;
    const uint8_t *p2 = s2;
    
    while (n--) {
        if (*p1 != *p2)
            return *p1 - *p2;
        p1++;
        p2++;
    }
    
    return 0;
}

void *memmove(void *dest, const void *src, size_t n)
{
    uint8_t *d = dest;
    const uint8_t *s = src;
    
    if (d < s) {
        /* Copy forward */
        while (n--)
            *d++ = *s++;
    } else if (d > s) {
        /* Copy backward */
        d += n;
        s += n;
        while (n--)
            *--d = *--s;
    }
    
    return dest;
}

size_t strlen(const char *s)
{
    const char *p = s;
    while (*p)
        p++;
    return p - s;
}

size_t strnlen(const char *s, size_t maxlen)
{
    const char *p = s;
    while (maxlen-- && *p)
        p++;
    return p - s;
}

char *strcpy(char *dest, const char *src)
{
    char *d = dest;
    while ((*d++ = *src++) != '\0')
        ;
    return dest;
}

char *strncpy(char *dest, const char *src, size_t n)
{
    char *d = dest;
    while (n && (*d++ = *src++) != '\0')
        n--;
    if (n)
        while (--n)
            *d++ = '\0';
    return dest;
}

int strcmp(const char *s1, const char *s2)
{
    while (*s1 && *s1 == *s2) {
        s1++;
        s2++;
    }
    return (unsigned char)*s1 - (unsigned char)*s2;
}

int strncmp(const char *s1, const char *s2, size_t n)
{
    while (n && *s1 && *s1 == *s2) {
        s1++;
        s2++;
        n--;
    }
    if (!n)
        return 0;
    return (unsigned char)*s1 - (unsigned char)*s2;
}

char *strchr(const char *s, int c)
{
    while (*s) {
        if (*s == (char)c)
            return (char *)s;
        s++;
    }
    return NULL;
}

char *strrchr(const char *s, int c)
{
    const char *last = NULL;
    while (*s) {
        if (*s == (char)c)
            last = s;
        s++;
    }
    return (char *)last;
}

char *strstr(const char *haystack, const char *needle)
{
    if (!*needle)
        return (char *)haystack;
    
    while (*haystack) {
        const char *h = haystack;
        const char *n = needle;
        
        while (*h && *n && *h == *n) {
            h++;
            n++;
        }
        
        if (!*n)
            return (char *)haystack;
        
        haystack++;
    }
    
    return NULL;
}

char *strcat(char *dest, const char *src)
{
    char *d = dest + strlen(dest);
    while ((*d++ = *src++) != '\0')
        ;
    return dest;
}

char *strncat(char *dest, const char *src, size_t n)
{
    char *d = dest + strlen(dest);
    while (n-- && (*d++ = *src++) != '\0')
        ;
    if (!n)
        *d = '\0';
    return dest;
}
