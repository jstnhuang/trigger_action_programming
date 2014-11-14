# Trigger action programming

Trigger action programming system for the PR2.

Make sure to install the [people](https://github.com/wg-perception/people) stack. This is meant to work with groovy, but it probably doesn't matter much. However, the version of ros-groovy-people in Ubuntu doesn't match the repo, so you'll want to clone the people stack into your workspace and build it yourself. Dependencies can be resolved with:

```
rosdep install --from-paths src --ignore-src --rosdistro=groovy -y
```
