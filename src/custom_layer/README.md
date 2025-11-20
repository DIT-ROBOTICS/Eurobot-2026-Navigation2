# Custom Layer

## Keepout Layer
Purpose: To switch specific zones between passable or not.

### How to use
Use topic `/keepout_zone` base on `std_msgs/msg/String`
- format example `{data: ABCDEFKLMN}`
    - one letter represent one zone, total 18 zones. (see the following picture)
    - the zones letter within `/keepout_zone` will be **not passable**, else will be passable.
    - for this example, zone `ABCDEFKLMN` will be **not passable**, zone `GHIJOPQR` will be passable.

### parameter settings
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      keepout_layer:
        inflation_length: 0.15   # distance over which cost is inflated
        cost_scaling_factor: 5.0   # higher value -> steeper cost increase
        keepout_expand_mode: 1   # 0: Circle, 1: Square
```
see more about the params [/navigation2_run/params/nav2_params_default.yaml](https://github.com/DIT-ROBOTICS/Eurobot-2026-Navigation2/blob/c15b2d0b84cd5adb5e536313d913c852af9cdcbd/src/navigation2_run/params/nav2_params_default.yaml#L194) 