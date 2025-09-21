#!/usr/bin/env node
'use strict';

// Node.js client equivalent of the Python rclpy client.
// Calls std_srvs/srv/Trigger service named 'identify_robot'.

const rclnodejs = require('rclnodejs');

async function main() {
  await rclnodejs.init();

  const node = rclnodejs.createNode('identify_client');
  const Trigger = rclnodejs.require('std_srvs').srv.Trigger;

  const client = node.createClient(Trigger, 'identify_robot');

  rclnodejs.spin(node);

  // Wait for the service to be available (polling every 1s)
  // client.waitForService(timeout) resolves to true if available within timeout
  // Fallback to retry loop if it returns false.
  // Some older versions may throw; catch to keep looping.
  // eslint-disable-next-line no-constant-condition
  while (true) {
    try {
      const ready = await client.waitForService(1000);
      if (ready) break;
    } catch (_) {
      // ignore and keep trying
    }
    node.getLogger().info('Service not available, waiting...');
  }

  const request = new Trigger.Request();

  // Use callback style for broad compatibility across rclnodejs versions
  await new Promise((resolve) => {
    client.sendRequest(request, (response) => {
      if (response) {
        node.getLogger().info(
          `Réponse: success=${response.success}, message='${response.message}'`
        );
      } else {
        node.getLogger().error('Service call failed');
      }
      resolve();
    });
  });

  // Give a brief moment for logs to flush
  setTimeout(() => {
    node.destroy();
    rclnodejs.shutdown();
  }, 10);
}

main().catch((err) => {
  // Last-resort error handler
  // eslint-disable-next-line no-console
  console.error(err);
  process.exit(1);
});

