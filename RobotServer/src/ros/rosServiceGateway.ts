import * as rclnodejs from "rclnodejs";

type AnyServiceType = rclnodejs.TypeClass<rclnodejs.ServiceTypeClassName>;

const DEFAULT_TIMEOUT_MS = 1000;

/**
 * Generic service-call wrapper around rclnodejs, keyed by service name.
 * This is where the `as unknown as rclnodejs.TypeClass<...>` cast for
 * project-local interface packages (e.g. autonav_msgs, which rclnodejs's
 * pre-generated type map doesn't know about until it's built+sourced) gets
 * centralized — one place instead of once per feature.
 */
export class RosServiceGateway {
  private readonly clients = new Map<string, rclnodejs.Client<AnyServiceType>>();

  constructor(private readonly node: rclnodejs.Node) {}

  async call<TReq, TRes>(
    type: string,
    serviceName: string,
    request: TReq,
    timeoutMs: number = DEFAULT_TIMEOUT_MS,
  ): Promise<TRes> {
    const client = this.getOrCreateClient(type, serviceName);

    const available = await client.waitForService(timeoutMs);
    if (!available) {
      throw new Error(`${serviceName} service is not available`);
    }

    const response = await client.sendRequestAsync(
      request as unknown as rclnodejs.ServiceRequestMessage<AnyServiceType>,
    );
    return response as unknown as TRes;
  }

  private getOrCreateClient(type: string, serviceName: string): rclnodejs.Client<AnyServiceType> {
    let client = this.clients.get(serviceName);
    if (!client) {
      client = this.node.createClient(type as unknown as AnyServiceType, serviceName);
      this.clients.set(serviceName, client);
    }
    return client;
  }
}

/** See TopicGateway in rosTopicGateway.ts for why this `Pick` exists. */
export type ServiceGateway = Pick<RosServiceGateway, "call">;
