#ifndef HAUL_SAFE_VECTOR_H
#define HAUL_SAFE_VECTOR_H

#include <stdlib.h>
#include <assert.h>

typedef struct safe_vector_t {
    void** items;

    int capacity;
    int stored;

    SemaphoreHandle_t xMutex;
} safe_vector_t;

void create_vector(safe_vector_t* vector, int initial_capacity);

void vector_push(safe_vector_t* vector, void* item);

void* vector_pop(safe_vector_t* vector);

void* vector_get(safe_vector_t* vector, int index);

int vector_empty(safe_vector_t* vector);

int vector_size(safe_vector_t* vector);

int vector_capacity(safe_vector_t* vector);

void free_vector(safe_vector_t* vector);

void free_vector_content(safe_vector_t* vector);
#ifdef HAUL_IMPLEMENTATION

// --- Helper Macro for error logging (optional but useful) ---
#define VECTOR_MUTEX_TIMEOUT pdMS_TO_TICKS(10)

void create_vector(safe_vector_t* vector, int initial_capacity) {
    assert(initial_capacity > 0);

    // 1. Create the Mutex first
    vector->xMutex = xSemaphoreCreateMutex();
    assert(vector->xMutex != NULL); // Ensure mutex creation succeeded

    vector->items = calloc(initial_capacity, sizeof(void*));
    assert(vector->items != NULL); // Ensure memory allocation succeeded

    vector->stored = 0;
    vector->capacity = initial_capacity;
}

void vector_push(safe_vector_t* vector, void* item) {
    // Acquire the lock (Wait indefinitely if needed, as push is essential)
    if (xSemaphoreTake(vector->xMutex, portMAX_DELAY) != pdTRUE) {
        // Should not happen with portMAX_DELAY, but for completeness
        return; 
    }

    // --- CRITICAL SECTION START ---
    if(vector->stored + 1 > vector->capacity) {
        // Expansion is a critical operation involving shared state and memory
        vector->capacity = vector->capacity * 2;
        void** new_items = realloc(vector->items, vector->capacity * sizeof(void*));
        
        // Handle realloc failure
        if (new_items == NULL) {
            // Restore original capacity before giving mutex if allocation fails
            vector->capacity = vector->capacity / 2; 
            xSemaphoreGive(vector->xMutex);
            return; 
        }
        vector->items = new_items;
    }

    vector->items[vector->stored] = item;
    ++vector->stored;
    // --- CRITICAL SECTION END ---

    // Release the lock
    xSemaphoreGive(vector->xMutex);
}

void* vector_pop(safe_vector_t* vector) {
    void* popped_value = NULL;

    // Acquire the lock
    if (xSemaphoreTake(vector->xMutex, portMAX_DELAY) != pdTRUE) {
        return NULL;
    }

    // --- CRITICAL SECTION START ---
    if(vector->stored == 0) {
        xSemaphoreGive(vector->xMutex); // Release before returning
        return NULL;
    }

    --vector->stored;
    popped_value = vector->items[vector->stored];
    
    // Check for shrinking capacity
    if(vector->stored > 10 && (vector->stored <= vector->capacity / 4)) { 
        // Note: Changed from /2 to /4 for better performance buffer
        // Shrinking is a critical operation
        vector->capacity = vector->capacity / 2;

        void** new_items = realloc(vector->items, vector->capacity * sizeof(void*));
        // If realloc fails when shrinking, we just keep the larger buffer, no need to revert capacity
        if (new_items != NULL) {
            vector->items = new_items;
        }
    }
    // --- CRITICAL SECTION END ---

    // Release the lock
    xSemaphoreGive(vector->xMutex);
    return popped_value;
}

void* vector_get(safe_vector_t* vector, int index) {
    void* item = NULL;

    // Acquire the lock (use a short timeout for simple read)
    if (xSemaphoreTake(vector->xMutex, VECTOR_MUTEX_TIMEOUT) != pdTRUE) {
        // Cannot access data safely, return NULL
        return NULL; 
    }

    // --- CRITICAL SECTION START ---
    if(vector->stored > index) {
        item = vector->items[index];
    }
    // --- CRITICAL SECTION END ---

    // Release the lock
    xSemaphoreGive(vector->xMutex);
    return item;
}

int vector_empty(safe_vector_t* vector) {
    int empty_status = 1; // Default to empty

    // Acquire the lock (use a short timeout for simple read)
    if (xSemaphoreTake(vector->xMutex, VECTOR_MUTEX_TIMEOUT) == pdTRUE) {
        empty_status = (vector->stored == 0);
        xSemaphoreGive(vector->xMutex);
    }
    return empty_status;
}

int vector_size(safe_vector_t* vector) {
    int size = 0;

    // Acquire the lock (use a short timeout for simple read)
    if (xSemaphoreTake(vector->xMutex, VECTOR_MUTEX_TIMEOUT) == pdTRUE) {
        size = vector->stored;
        xSemaphoreGive(vector->xMutex);
    }
    return size;
}

int vector_capacity(safe_vector_t* vector) {
    int capacity = 0;

    // Acquire the lock (use a short timeout for simple read)
    if (xSemaphoreTake(vector->xMutex, VECTOR_MUTEX_TIMEOUT) == pdTRUE) {
        capacity = vector->capacity;
        xSemaphoreGive(vector->xMutex);
    }
    return capacity;
}

void free_vector(safe_vector_t* vector) {
    // Acquire the lock before freeing resources
    if (xSemaphoreTake(vector->xMutex, portMAX_DELAY) == pdTRUE) {
        free(vector->items);
        vector->items = NULL;
        xSemaphoreGive(vector->xMutex); // Release after freeing resources
    }
    
    // Delete the Mutex itself
    vSemaphoreDelete(vector->xMutex);
}

void free_vector_content(safe_vector_t* vector) {
    // Acquire the lock before iterating and freeing content
    if (xSemaphoreTake(vector->xMutex, portMAX_DELAY) == pdTRUE) {
        for(int i = 0; i < vector->stored; ++i) {
            free(vector->items[i]);
        }
        // It's usually expected that free_vector() is called afterwards, 
        // but we release the lock here so other tasks can proceed.
        xSemaphoreGive(vector->xMutex); 
    }
}

#endif

#endif