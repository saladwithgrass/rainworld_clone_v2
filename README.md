## Scope of the project
This is a game about a procedurally animated spider.
Style is something a-la rain-world.

Spider uses procedural animation.

## Spider animation
Spider's movement consists of two main components:
1) Leg IK
2) Comfort points

### Leg Inverse Kinematics
Just basic gradient descent IK

### Comfort Points
Spider has some comfort points. 
When spider moves, they try to remain in the same position, but when too far, they are moved forward some amount.
When each point is moved, it should be moved to the new comfortable position.

