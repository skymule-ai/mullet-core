# mullet_core Documentation

This folder contains source documentation for the `mullet_core` ROS 2 package.
The docs are plain Markdown so they stay reviewable with the C++ state machine
and do not require a separate frontend build.

## Reading Order

1. [State machine](state-machine.md): states, transitions, and guardrails.
2. [Actions](actions.md): action and service contracts.
3. [Configuration](configuration.md): parameters and defaults.
4. [PX4 interface](px4-interface.md): topics, QoS, PX4 commands, and
   multi-instance mapping.
5. [Testing](testing.md): build, test, and scenario coverage.

## Design Notes

Proposals not yet implemented. They become part of the docs above when they land.

- [Map-frame missions](map-frame-missions.md): flying missions drawn on a prior
  map, with the map correction applied to the targets, never to PX4's estimate.

## Maintenance Rule

When state-machine behavior, public actions, launch arguments, or configuration
defaults change, update these docs in the same change. Keep the docs focused on
operator-visible behavior and source references; avoid duplicating every branch
of implementation code.
