#ifndef PART_INT_H
#define PART_INT_H

#include "particle_common.h"
#include <unistd.h>

static ParticleError Particle_EulerIntegrate(Particle *particle, real outDuration);

static Vector calculateK(Particle *particle, real duration);
static ParticleError Particle_RKIntegrate(Particle *particle, real duration);

ParticleError Particle_Integrate(Particle *particle, real duration);
ParticleError SimulateParticles(Particle *particleArray,
                                const size_t numOfParticles,
                                const real frameRate);

static ParticleError Particle_EulerIntegrate(Particle *particle, real outDuration)
{
    if (!particle)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (outDuration <= 0.0)
    {
        return PARTICLE_ERROR_INVALID_DURATION;
    }

    /* const int num_threads = omp_get_num_procs();
    omp_set_num_threads(num_threads);

    const size_t chunk_size = (particle->forceCount / num_threads) + 1; */

    omp_set_num_threads(4);

    real duration = outDuration / N_STEPS;

    for (size_t i = 0; i < N_STEPS; i++)
    {
        addScaled(&particle->position, &particle->velocity, 1.0, duration);

        Vector totalForce = nullVectorDef();

        /* #pragma omp parallel for schedule(dynamic, chunk_size) */
        /* #pragma omp parallel for */
        for (size_t i = 0; i < particle->forceCount; i++)
        {
            ForceGenerator *generator = &particle->forceRegistry[i];
            if (!generator->isActive)
                continue;

            if (particle->time >= generator->startTime &&
                particle->time <= generator->endTime)
            {
                Vector force = generator->function(particle, generator->parameters);
                /* #pragma omp critical */
                vecAdd(&particle->resultantForce, &force);
            }
        }

        bool isStatic;
        ParticleError error = Particle_IsStatic(particle, &isStatic);
        if (error != PARTICLE_SUCCESS)
        {
            return error;
        }

        if (!isStatic)
        {
            Vector accFromForce = particle->resultantForce;
            scale(&accFromForce, particle->inverseMass);
            vecAdd(&particle->acceleration, &accFromForce);
        }

        real dampingFactor = pow(particle->damping, duration);
        addScaled(&particle->velocity, &particle->acceleration, dampingFactor, duration);

        particle->resultantForce = nullVectorDef();
        particle->time += duration;
        particle->acceleration = nullVectorDef();
    }

    return PARTICLE_SUCCESS;
}

static Vector calculateK(Particle *particle, real duration)
{
    Vector totalForce = nullVectorDef();

    /* const int num_threads = omp_get_num_procs();
    omp_set_num_threads(num_threads);

    const size_t chunk_size = (particle->forceCount / num_threads) + 1; */

    /*   omp_set_num_threads(4);

  #pragma omp parallel for */
    for (size_t i = 0; i < particle->forceCount; i++)
    {
        ForceGenerator *generator = &particle->forceRegistry[i];
        if (!generator->isActive)
            continue;

        if (particle->time >= generator->startTime &&
            particle->time <= generator->endTime)
        {
            Vector force = generator->function(particle, generator->parameters);
            /* #pragma omp critical */
            vecAdd(&totalForce, &force);
        }
    }

    bool isStatic = false;
    Particle_IsStatic(particle, &isStatic);

    if (isStatic)
    {
        return nullVectorDef();
    }

    scale(&totalForce, particle->inverseMass * duration);
    return totalForce;
}

static ParticleError Particle_RKIntegrate(Particle *particle, real duration)
{
    if (!particle)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (duration <= 0.0)
    {
        return PARTICLE_ERROR_INVALID_DURATION;
    }

    Vector v_k1, v_k2, v_k3, v_k4;
    Vector x_k1, x_k2, x_k3, x_k4;
    Vector initVel = particle->velocity;
    Vector initPos = particle->position;
    real initTime = particle->time;

    x_k1 = particle->velocity;
    scale(&x_k1, duration);

    v_k1 = calculateK(particle, duration);

    particle->time += duration * 0.5;
    addScaled(&(particle->position), &initVel, 1.0, 0.5 * duration);
    addScaled(&(particle->velocity), &v_k1, 1, 0.5);

    x_k2 = particle->velocity;
    scale(&x_k2, duration);

    v_k2 = calculateK(particle, duration);

    particle->velocity = initVel;
    addScaled(&(particle->velocity), &v_k2, 1, 0.5);

    x_k3 = particle->velocity;
    scale(&x_k3, duration);

    v_k3 = calculateK(particle, duration);

    particle->time += duration * 0.5;
    addScaled(&(particle->position), &initVel, 1.0, 0.5 * duration);
    particle->velocity = initVel;
    vecAdd(&(particle->velocity), &v_k3);

    x_k4 = particle->velocity;
    scale(&x_k4, duration);

    v_k4 = calculateK(particle, duration);

    particle->position = initPos;
    particle->velocity = initVel;

    addScaled(&(particle->velocity), &v_k1, 1.0, 1.0 / 6);
    addScaled(&(particle->velocity), &v_k2, 1.0, 2.0 / 6);
    addScaled(&(particle->velocity), &v_k3, 1.0, 2.0 / 6);
    addScaled(&(particle->velocity), &v_k4, 1.0, 1.0 / 6);

    addScaled(&(particle->position), &x_k1, 1.0, 1.0 / 6);
    addScaled(&(particle->position), &x_k2, 1.0, 2.0 / 6);
    addScaled(&(particle->position), &x_k3, 1.0, 2.0 / 6);
    addScaled(&(particle->position), &x_k4, 1.0, 1.0 / 6);

    return PARTICLE_SUCCESS;
}

static inline uint64_t get_microseconds(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static inline void precise_sleep_us(uint64_t microseconds)
{
    if (microseconds > 0)
    {
        const uint64_t CHUNK_SIZE = 1000;
        while (microseconds > CHUNK_SIZE)
        {
            usleep(CHUNK_SIZE);
            microseconds -= CHUNK_SIZE;
        }
        if (microseconds > 0)
        {
            usleep(microseconds);
        }
    }
}

ParticleError Particle_Integrate(Particle *particle, real duration)
{
#ifdef RK4
    return Particle_RKIntegrate(particle, duration);
#else
    return Particle_EulerIntegrate(particle, duration);
#endif
}

ParticleError SimulateParticles(Particle *particleArray,
                                const size_t numOfParticles,
                                const real frameRate)
{
    if (!particleArray)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
    if (frameRate <= 0.0)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
    if (numOfParticles == 0)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    const real frameDuration = 1.0 / frameRate;
    const uint64_t frameDurationUs = (uint64_t)(frameDuration * 1000000.0);

    /* const int num_threads = omp_get_num_procs();
    omp_set_num_threads(num_threads);

    const size_t chunk_size = (numOfParticles / num_threads) + 1; */

    omp_set_num_threads(THREAD_NO);
    const size_t chunk_size = (numOfParticles / THREAD_NO) + 1;

#pragma omp parallel
    {
    }

    uint64_t last_frame_time = get_microseconds();
    uint64_t current_time;
    uint64_t elapsed_time;

    while (true)
    {
        current_time = get_microseconds();

#pragma omp parallel for schedule(dynamic, chunk_size)
        for (size_t i = 0; i < numOfParticles; i++)
        {
            if (!(&particleArray[i]))
            {
                continue;
            }

            Particle_Integrate(&particleArray[i], frameDuration);
        }

        current_time = get_microseconds();
        elapsed_time = current_time - last_frame_time;

        if (elapsed_time < frameDurationUs)
        {
            precise_sleep_us(frameDurationUs - elapsed_time);
            last_frame_time = last_frame_time + frameDurationUs;
        }
        else
        {
            last_frame_time = current_time;
        }
    }

    return PARTICLE_SUCCESS;
}

#endif