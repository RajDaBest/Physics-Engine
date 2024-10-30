#include "../src/particle.h"
#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080
#define CIRCLE_RADIUS 10
#define MAX_PARTICLES 100000
#define TARGET_FPS 120
#define FRAME_TIME (1.0 / TARGET_FPS)
#define PARTICLES_PER_THREAD 10
#define MAX_THREADS 16

// Spring constants
#define SPRING_COEFFICIENT 50.0f
#define SPRING_REST_LENGTH 20.0f
#define DAMPING_COEFFICIENT 0.1f

typedef struct
{
    Particle *particle;
    bool active;
    int pairIndex;
} ParticleInstance;

typedef struct
{
    ParticleInstance *particles;
    int startIndex;
    int endIndex;
    real deltaTime;
    int windowHeight;
    pthread_mutex_t *mutex;
} ThreadData;

// Global variables
static DragCoefficients dragCoeffs = {0.05, 0.005};
static pthread_t threads[MAX_THREADS];
static ThreadData threadData[MAX_THREADS];
static int activeThreads = 0;
static pthread_mutex_t particleMutex = PTHREAD_MUTEX_INITIALIZER;

// Previous helper functions remain the same
static void renderCircle(SDL_Renderer *renderer, int x, int y, int radius)
{
    for (int w = 0; w < radius * 2; w++)
    {
        for (int h = 0; h < radius * 2; h++)
        {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius))
            {
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
            }
        }
    }
}

static void renderSpring(SDL_Renderer *renderer, Vector p1, Vector p2, int windowHeight)
{
    int x1 = (int)p1.x;
    int y1 = windowHeight - (int)p1.y;
    int x2 = (int)p2.x;
    int y2 = windowHeight - (int)p2.y;
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

static Vector getRandomPosition(void)
{
    return vectorDef(
        CIRCLE_RADIUS + (rand() % (WINDOW_WIDTH - 2 * CIRCLE_RADIUS)),
        CIRCLE_RADIUS + (rand() % (WINDOW_HEIGHT - 2 * CIRCLE_RADIUS)),
        0.0);
}

static Vector getRandomVelocity(void)
{
    return vectorDef(
        -200.0f + (rand() % 401),
        -200.0f + (rand() % 401),
        0.0);
}

// Thread function for updating particles
static void *updateParticles(void *arg)
{
    ThreadData *data = (ThreadData *)arg;

    for (int i = data->startIndex; i < data->endIndex; i++)
    {
        pthread_mutex_lock(data->mutex);
        if (data->particles[i].active && data->particles[i].particle)
        {
            ParticleError error = Particle_Integrate(data->particles[i].particle, data->deltaTime);

            if (error == PARTICLE_SUCCESS)
            {
                Vector pos = data->particles[i].particle->position;
                if (pos.x < -CIRCLE_RADIUS || pos.x > WINDOW_WIDTH + CIRCLE_RADIUS ||
                    pos.y < -CIRCLE_RADIUS || pos.y > WINDOW_HEIGHT + CIRCLE_RADIUS)
                {

                    if (data->particles[i].pairIndex >= 0)
                    {
                        int pairIdx = data->particles[i].pairIndex;
                        if (data->particles[pairIdx].active)
                        {
                            Particle_Destroy(data->particles[pairIdx].particle);
                            data->particles[pairIdx].particle = NULL;
                            data->particles[pairIdx].active = false;
                        }
                    }
                    Particle_Destroy(data->particles[i].particle);
                    data->particles[i].particle = NULL;
                    data->particles[i].active = false;
                }
            }
        }
        pthread_mutex_unlock(data->mutex);
    }
    return NULL;
}

// Function to manage thread creation and assignment
static void updateParticlesThreaded(ParticleInstance *particles, int particleCount, real deltaTime)
{
    int particlesPerThread = PARTICLES_PER_THREAD;
    int requiredThreads = (particleCount + particlesPerThread - 1) / particlesPerThread;
    int threadsToUse = requiredThreads < MAX_THREADS ? requiredThreads : MAX_THREADS;

    // Distribute particles among threads
    for (int i = 0; i < threadsToUse; i++)
    {
        threadData[i].particles = particles;
        threadData[i].startIndex = i * (particleCount / threadsToUse);
        threadData[i].endIndex = (i == threadsToUse - 1) ? particleCount : (i + 1) * (particleCount / threadsToUse);
        threadData[i].deltaTime = deltaTime;
        threadData[i].mutex = &particleMutex;

        pthread_create(&threads[i], NULL, updateParticles, &threadData[i]);
    }

    // Wait for all threads to complete
    for (int i = 0; i < threadsToUse; i++)
    {
        pthread_join(threads[i], NULL);
    }
}

static ParticleError createParticlePair(ParticleInstance *particles, int *particleCount)
{
    if (*particleCount >= MAX_PARTICLES - 1)
        return PARTICLE_ERROR_INVALID_PARAM;

    Vector pos1 = getRandomPosition();
    Vector vel1 = getRandomVelocity();
    Vector initAcc = nullVectorDef();

    Particle *p1 = Particle_Create(
        pos1, vel1, initAcc, 2.0, 0.99, 0.0);

    if (!p1)
        return particleErrno;

    Vector pos2 = getRandomPosition();
    Vector vel2 = getRandomVelocity();

    Particle *p2 = Particle_Create(
        pos2, vel2, initAcc, 2.0, 0.99, 0.0);

    if (!p2)
    {
        Particle_Destroy(p1);
        return particleErrno;
    }

    ParticleError error;
    if ((error = Particle_AddGrav(p2)) != PARTICLE_SUCCESS ||
        (error = Particle_AddDrag(p2, &dragCoeffs)) != PARTICLE_SUCCESS ||
        (error = Particle_AddGrav(p1)) != PARTICLE_SUCCESS ||
        (error = Particle_AddDrag(p1, &dragCoeffs)) != PARTICLE_SUCCESS)
    {
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    }

    SpringParameters *spring = malloc(sizeof(SpringParameters));
    if (!spring)
    {
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return PARTICLE_ERROR_MEMORY;
    }

    error = buildSpringParameters(p1, p2, SPRING_COEFFICIENT, SPRING_REST_LENGTH,
                                  DAMPING_COEFFICIENT, spring);
    if (error != PARTICLE_SUCCESS)
    {
        free(spring);
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    }

    error = Particle_AddElasticBungee(p1, p2, spring, 0.0, INFINITY);
    if (error != PARTICLE_SUCCESS)
    {
        free(spring);
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    }

    pthread_mutex_lock(&particleMutex);
    particles[*particleCount].particle = p1;
    particles[*particleCount].active = true;
    particles[*particleCount].pairIndex = *particleCount + 1;

    particles[*particleCount + 1].particle = p2;
    particles[*particleCount + 1].active = true;
    particles[*particleCount + 1].pairIndex = *particleCount;

    *particleCount += 2;
    pthread_mutex_unlock(&particleMutex);

    return PARTICLE_SUCCESS;
}

static void cleanupParticles(ParticleInstance *particles, int count)
{
    pthread_mutex_lock(&particleMutex);
    for (int i = 0; i < count; i++)
    {
        if (particles[i].active && particles[i].particle)
        {
            Particle_Destroy(particles[i].particle);
            particles[i].particle = NULL;
            particles[i].active = false;
        }
    }
    pthread_mutex_unlock(&particleMutex);
}

int main(void)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "Threaded Particle Spring Simulation",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN);

    if (!window)
    {
        fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(
        window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (!renderer)
    {
        fprintf(stderr, "Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    srand((unsigned int)time(NULL));

    ParticleInstance particles[MAX_PARTICLES] = {0};
    int particleCount = 0;
    bool running = true;
    SDL_Event event;

    Uint32 lastTime = SDL_GetTicks();

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.sym == SDLK_a)
                {
                    ParticleError error = createParticlePair(particles, &particleCount);
                    if (error != PARTICLE_SUCCESS)
                    {
                        fprintf(stderr, "Failed to create particle pair: error %d\n", error);
                    }
                }
            }
        }

        Uint32 currentTime = SDL_GetTicks();
        real deltaTime = FRAME_TIME;
        lastTime = currentTime;

        // Update physics using thread pool
        updateParticlesThreaded(particles, particleCount, deltaTime);

        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Draw springs
        SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);
        for (int i = 0; i < particleCount; i++)
        {
            pthread_mutex_lock(&particleMutex);
            if (particles[i].active && particles[i].pairIndex >= 0)
            {
                int pairIdx = particles[i].pairIndex;
                if (particles[pairIdx].active)
                {
                    renderSpring(renderer,
                                 particles[i].particle->position,
                                 particles[pairIdx].particle->position,
                                 WINDOW_HEIGHT);
                }
            }
            pthread_mutex_unlock(&particleMutex);
        }

        // Draw particles
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for (int i = 0; i < particleCount; i++)
        {
            pthread_mutex_lock(&particleMutex);
            if (particles[i].active && particles[i].particle)
            {
                int screenX = (int)particles[i].particle->position.x;
                int screenY = WINDOW_HEIGHT - (int)particles[i].particle->position.y;
                renderCircle(renderer, screenX, screenY, CIRCLE_RADIUS);
            }
            pthread_mutex_unlock(&particleMutex);
        }

        SDL_RenderPresent(renderer);
    }

    cleanupParticles(particles, particleCount);
    pthread_mutex_destroy(&particleMutex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}