## delete pos="x x x" in all joints;

```XML
<body name="torso">
```

to 

```XML
<body name="torso">
```

## delete floor (pybullet has floor by default);

delete this line:

```xml
<geom conaffinity="1" condim="3" material="grid" name="floor" pos="0 0 0" size="20 20 0.125" type="plane" />
```

## add ignore to joint root, adjust the default height for torso;


```xml
<joint armature="0" axis="1 0 0" damping="0" limited="false" name="ignore_rootx" pos="0 0 0" stiffness="0" type="slide"/>
<joint armature="0" axis="0 0 1" damping="0" limited="false" name="ignore_rootz" pos="0 0 0" ref="1.25" stiffness="0" type="slide"/>
<joint armature="0" axis="0 1 0" damping="0" limited="false" name="ignore_rooty" pos="0 0 1.25" stiffness="0" type="hinge"/>
```

to

```xml
<joint armature="0" axis="1 0 0" damping="0" limited="false" name="ignore_rootx" pos="0 0 0" stiffness="0" type="slide"/>
<joint armature="0" axis="0 0 1" damping="0" limited="false" name="ignore_rootz" pos="0 0 0" ref="1.25" stiffness="0" type="slide"/>
<joint armature="0" axis="0 1 0" damping="0" limited="false" name="ignore_rooty" pos="0 0 -0.5" stiffness="0" type="hinge"/>
```