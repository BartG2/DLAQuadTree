#include <chrono>
#include <random>
#include "raylib.h"
#include <iostream>
#include <vector>
#include <future>
#include <omp.h>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <thread>
#include <functional>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <list>

std::mt19937 CreateGeneratorWithTimeSeed();
float RandomFloat(float min, float max, std::mt19937& rng);
bool Contains(const Rectangle& r1, const Rectangle& r2);

//-------------------------------------------------------------------------------------------------------------------------------------------------------

constexpr int screenWidth = 2560, screenHeight = 1440, numThreads = 2;
int maxTreeDepth = 6, minParticlesToDivide = 500;
const float collisionThreshold = 1.1f, minimumStickDistance = 1.0f;
float stickingProbability = 1.0f;

std::mt19937 rng = CreateGeneratorWithTimeSeed();

//-------------------------------------------------------------------------------------------------------------------------------------------------------

class Particle {
public:
    Vector2 pos;
    Vector2 v;
    Vector2 a;
    Color color;
    bool isStuck;

    Particle(Vector2 position, Color col, Vector2 velocity = {0, 0}, Vector2 acceleration = {0, 0}){
        pos = position;
        v = velocity;
        a = acceleration;
        color = col;
        isStuck = false;
    }

    void RandomWalk(float stepSize, int numSteps) {
        if(!isStuck){
            for (int i = 0; i < numSteps; i++) {
                float dx = RandomFloat(-1, 1, rng);
                float dy = RandomFloat(-1, 1, rng);

                float newX = pos.x + dx * stepSize;
                float newY = pos.y + dy * stepSize;

                // Check if particle is out of bounds and correct position
                if (newX < 0) {
                    newX = 0;
                }
                else if (newX > screenWidth) {
                    newX = screenWidth;
                }
                if (newY < 0) {
                    newY = 0;
                }
                else if (newY > screenHeight) {
                    newY = screenHeight;
                }

                pos.x = newX;
                pos.y = newY;
            }
        }
    }

    void updatePosition(){
        pos.x += v.x;
        pos.y += v.y;

        v.x += a.x;
        v.y += a.y;
    }

};

class Timer{
public:
    std::chrono::time_point<std::chrono::high_resolution_clock> startPoint;

    Timer(){
        startPoint = std::chrono::high_resolution_clock::now();
    }

    ~Timer(){
        stop();
    }

    double stop(){
        auto endPoint = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(startPoint).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endPoint).time_since_epoch().count();

        auto duration = end - start;
        double ms = duration * 0.001;

        std::cout << "ms: " << ms << std::endl;

        return ms;
    }
};

template <typename OBJECT_TYPE>
class QuadTree{
public:
    int currentDepth;
    Rectangle currentSize;
    std::vector<std::pair<Rectangle, OBJECT_TYPE>> objects;
    std::array<std::shared_ptr<QuadTree<OBJECT_TYPE>>, 4> children{};
    std::array<Rectangle, 4> childAreas{};

    QuadTree(const Rectangle& size = Rectangle{0, 0, 1, 1}, const int setDepth = 0){
        currentDepth = setDepth;
        resize(size);
    }

    void resize(const Rectangle& newSize){
        clear();
        currentSize = newSize;

        float newWidth = currentSize.width / 2.0f, newHeight = currentSize.height / 2.0f;
        float x = currentSize.x, y = currentSize.y;

        childAreas = {
            Rectangle{x + newWidth, y, newWidth, newHeight},
            Rectangle{x, y, newWidth, newHeight},
            Rectangle{x, y + newHeight, newWidth, newHeight},
            Rectangle{x + newWidth, y + newHeight, newWidth, newHeight}
        };        
    }

    void clear(){
        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->clear();
            }
            children[i].reset();
        }
    }

    int size() const{
        int count = objects.size();

        for(int i = 0 ; i < 4; i++){
            if(children[i]){
                count += children[i]->size();
            }
        }

        return count;
    }

    void insert(const OBJECT_TYPE& newItem, const Rectangle& itemSize){
        for(int i = 0 ; i < 4; i++){
            if(CheckCollisionRecs(itemSize, childAreas[i])){
                if(currentDepth + 1 < maxTreeDepth){
                    if(!children[i]){
                        children[i] = std::make_shared<QuadTree<OBJECT_TYPE>>(childAreas[i], currentDepth + 1);
                    }
                    children[i]->insert(newItem, itemSize);
                    return;
                }
            }
        }

        //didn't fit in children, so must go here
        std::pair<Rectangle, OBJECT_TYPE> temp = {itemSize, newItem};
        objects.push_back(temp);
    }

    std::list<OBJECT_TYPE> search(Vector2& center, float radius, bool removeSearched){
        std::list<OBJECT_TYPE> result;

        // Check if the search area intersects the QuadTree node's boundary
        if(!CheckCollisionCircleRec(center, radius, currentSize)) {
            return result;
        }

        // If this node has objects, add the ones within the search area to the result list
        for(unsigned int i = 0; i < objects.size(); i++){
            if(CheckCollisionCircleRec(center, radius, objects[i].first)){
                result.push_back(objects[i].second);
                if(removeSearched){
                    objects.erase(objects.begin() + i);
                }
            }
        }

        // Recursively search the children
        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->search(center, radius, removeSearched);
                result.splice(result.end(), childResult);
            }
        }

        return result;
    }

    std::vector<OBJECT_TYPE> returnAll(int depth){
        std::vector<OBJECT_TYPE> result;

        if(currentDepth >= depth){
            result.insert(result.end(), objects.begin(), objects.end());
        }

        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->returnAll(depth);
                result.insert(result.end(), childResult.begin(), childResult.end());
            }
        }

        return result;
    }

    void draw() const{
        for(const auto& particle : objects){
            DrawPixelV(particle.second.pos, particle.second.color);
        }

        //DrawRectangleLinesEx(currentSize, 0.7, GREEN);

        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->draw();
            }
        }
    }

};

template <typename OBJECT_TYPE>
class QuadTreeContainer{
    using QTContainer = std::list<OBJECT_TYPE>;
public:
    QTContainer allObjects;
    QuadTree<typename QTContainer::iterator> root;

    QuadTreeContainer(const Rectangle& size = Rectangle{0, 0, 0, 0}, const int nDepth = 0) : root(size, nDepth)
    {}

    void resize(const Rectangle& newSize){
        root.resize(newSize);
    }

    int size() const{
        return allObjects.size();
    }

    bool empty() const{
        return allObjects.empty();
    }

    void clear(){
        root.clear();
        allObjects.clear();
    }

    typename QTContainer::iterator begin(){
        return allObjects.begin();
    }

    typename QTContainer::iterator end(){
        return allObjects.end();
    }

    typename QTContainer::iterator cbegin(){
        return allObjects.cbegin();
    }

    typename QTContainer::iterator cend(){
        return allObjects.cend();
    }

    void insert(const OBJECT_TYPE& object, const Rectangle & objectSize){
        allObjects.push_back(object);
        root.insert(std::prev(allObjects.end()), objectSize);
    }

    std::list<typename QTContainer::iterator> search(Vector2& center, float radius, bool removeSearched){
        return root.search(center, radius, removeSearched);
    }

    void draw() const{
        for(const auto& p : allObjects){
            DrawPixelV(p.pos, p.color);
        }
    }
};

//-------------------------------------------------------------------------------------------------------------------------------------------------------

std::vector<Particle> freeParticles, aggregateParticles;

//-------------------------------------------------------------------------------------------------------------------------------------------------------

std::mt19937 CreateGeneratorWithTimeSeed() {
    // Get the current time in nanoseconds
    auto now = std::chrono::high_resolution_clock::now();
    auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();

    // Create a new mt19937 generator and seed it with the current time in nanoseconds
    std::mt19937 gen(static_cast<unsigned int>(nanos));
    return gen;
}

float RandomFloat(float min, float max, std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

float vector2distance(Vector2 v1, Vector2 v2) {
    float dx = v2.x - v1.x;
    float dy = v2.y - v1.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Particle> CreateCircle(int numParticles, Color col, Vector2 center, float radius){
    float degreeIncrement = 360.0f/(float)numParticles;
    std::vector<Particle> particles;

    for(float i = 0; i < 360; i += degreeIncrement){
        float x = radius * cos(i) + center.x;
        float y = radius * sin(i) + center.y;
        Particle p({x, y}, col);
        particles.push_back(p);
    }

    return particles;
}

bool vectorsEqual(Vector2 v1, Vector2 v2){
    if(v1.x == v2.x && v1.y == v2.y){
        return true;
    }
    else{
        return false;
    }
}

void primitiveCollisionCheck(){
    for(unsigned int i = 0; i < aggregateParticles.size(); i++){
        for(unsigned int j = 0; j < freeParticles.size(); j++){
            if(CheckCollisionPointCircle(freeParticles[j].pos, aggregateParticles[i].pos, collisionThreshold)){
                freeParticles[j].color = WHITE;
                aggregateParticles.push_back(freeParticles[j]);
                freeParticles.erase(freeParticles.begin() + j);
            }
        }
    }
}

std::vector<Particle> collisionCheck(QuadTreeContainer<Particle> qtc){
    std::vector<Particle> failedCollisions;
    failedCollisions.reserve(2);

    for(auto& aggregateParticle : aggregateParticles){
        for(auto p : qtc.search(aggregateParticle.pos, collisionThreshold, true)){
            float dist = vector2distance(p->pos, aggregateParticle.pos);
            if(dist >= minimumStickDistance){
                p->color = WHITE;
                aggregateParticles.push_back(*p);
            }
            else{
                p->color = RED;
                p->pos.x = screenWidth;
                failedCollisions.push_back(*p);
            }
        }
    }

    return failedCollisions;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------

void Initialize(){
    InitWindow(screenWidth, screenHeight, "DLA, hopefully");
    SetTargetFPS(100);

    constexpr int startingNumParticles = 10, startingRadius = 100;
    const Color startingColor = RED;
    const Vector2 startingCenter = {screenWidth / 2, screenHeight / 2};

    freeParticles = CreateCircle(startingNumParticles, startingColor, startingCenter, startingRadius);
    aggregateParticles = {1, Particle({screenWidth / 2.0, screenHeight / 2.0}, WHITE)};
}

void DrawParticlesVector(const std::vector<Particle>& particles){
    for(unsigned int i = 0; i < particles.size(); i++){
        DrawPixelV(particles[i].pos, particles[i].color);
    }
}

void RandomWalkAll(std::vector<Particle>& particles){
    for(auto& p : particles){
        p.RandomWalk(2, 1);
    }
}

void ConcentricCircles(int frameCount){
    if(frameCount / 5 < screenHeight / 2 && frameCount % 500 == 0){
        std::vector<Particle> fp2 = CreateCircle(300 * (1 + frameCount / 50),RED,{screenWidth/2.0, screenHeight/2.0}, 50 + frameCount / 5);
        freeParticles.insert(freeParticles.end(), fp2.begin(), fp2.end());
    }
}

QuadTreeContainer<Particle> initializeQTC(){
    QuadTreeContainer<Particle> qtc(Rectangle{0, 0, screenWidth, screenHeight}, 0);

    for(const auto& p : freeParticles){
        qtc.insert(p, Rectangle{p.pos.x, p.pos.y, 0, 0});
    }

    return qtc;
}

float findMaxAggregateRadius(){
    float maxAggregateRadius = 0;
    for(auto& p : aggregateParticles){
        if(vector2distance(p.pos, {screenWidth/2.0f, screenHeight/2.0f}) > maxAggregateRadius){
            maxAggregateRadius = vector2distance(p.pos, {screenWidth/2.0f, screenHeight/2.0f});
        }
    }
    return maxAggregateRadius;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------

int main(){
    Initialize();
    
    for(int frameCount = 0; !WindowShouldClose(); frameCount++){

        ConcentricCircles(frameCount);
        RandomWalkAll(freeParticles);

        QuadTreeContainer qtc = initializeQTC();

        //std::vector<Particle> failedCollisions = collisionCheck(qtc);

        //auto n = qt.returnAll(0);
        /*
        for(unsigned int i = 0; i < failedCollisions.size(); i++){
            freeParticles.push_back(failedCollisions[i]);
            freeParticles[freeParticles.size()].RandomWalk(2,1);
        }*/

        BeginDrawing();
        {
            ClearBackground(BLACK);
            DrawFPS(10,10);
            DrawText(TextFormat("%d freeparticles, and %d aggregate particles\t %d total particles", freeParticles.size(), aggregateParticles.size(), freeParticles.size() + aggregateParticles.size()), 10, 30, 10, GREEN);

            DrawParticlesVector(aggregateParticles);
            qtc.draw();
            //DrawCircleLines(screenWidth / 2.0f, screenHeight / 2.0f, maxAggregateRadius, ORANGE);
        }
        EndDrawing();

        if(frameCount % 5000 == 0){
            stickingProbability -= 0.01;
            std::cout << stickingProbability << std::endl;
        }

    }

    return 0;
}
