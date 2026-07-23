# GFS-X transport patch and provenance

This embedded package is derived from Unity Technologies'
`ROS-TCP-Connector`, upstream commit
`c27f00c6cf750d2d0564349b3039d19aa3925e7c`, package version
`0.7.0-preview`.

Upstream source: <https://github.com/Unity-Technologies/ROS-TCP-Connector>

It is distributed under Apache License 2.0. The exact upstream license is
included as `LICENSE` in this directory.

This package is embedded so the real-robot ROS link keeps two local fixes:

- `TcpClient.NoDelay = true` prevents small heartbeat and command messages
  from arriving in delayed bursts.
- publisher registrations use the active `NetworkStream` during reconnects,
  matching the subscriber/service registration paths.

Modified source files:

- `Runtime/TcpConnector/ROSConnection.cs`
- `Runtime/TcpConnector/RosTopicState.cs`

The project manifest and lock refer to this directory as an embedded package,
so Unity will not silently replace the patched transport with an upstream Git
checkout.
