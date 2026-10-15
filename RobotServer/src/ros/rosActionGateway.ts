import * as rclnodejs from "rclnodejs";

type AnyActionType = rclnodejs.TypeClass<rclnodejs.ActionTypeClassName>;

const DEFAULT_TIMEOUT_MS = 1000;

/**
 * Generic action-call wrapper around rclnodejs, keyed by action name.
 */
export class RosActionGateway {
  private readonly clients = new Map<string, rclnodejs.ActionClient<AnyActionType>>();

  constructor(private readonly node: rclnodejs.Node) {}

  async sendGoal<TGoal, TFeedback, TResult>(
    type: string,
    actionName: string,
    goal: TGoal,
    onFeedback?: (feedback: TFeedback) => void,
    timeoutMs: number = DEFAULT_TIMEOUT_MS,
  ): Promise<{ handle: rclnodejs.ClientGoalHandle<any>; result: Promise<TResult> }> {
    const client = this.getOrCreateClient(type, actionName);

    const available = await client.waitForServer(timeoutMs);
    if (!available) {
      throw new Error(`${actionName} action server is not available`);
    }

    const handle = await client.sendGoal(
      goal as unknown as rclnodejs.ActionGoal<AnyActionType>,
      onFeedback
        ? (feedbackMsg: rclnodejs.ActionFeedback<AnyActionType>) => onFeedback(feedbackMsg as unknown as TFeedback)
        : undefined,
    );

    const result = handle.getResult() as unknown as Promise<TResult>;
    return { handle, result };
  }

  async cancelGoal(handle: rclnodejs.ClientGoalHandle<any>): Promise<void> {
    await handle.cancelGoal();
  }

  private getOrCreateClient(type: string, actionName: string): rclnodejs.ActionClient<AnyActionType> {
    let client = this.clients.get(actionName);
    if (!client) {
      client = new rclnodejs.ActionClient(this.node, type as unknown as AnyActionType, actionName);
      this.clients.set(actionName, client);
    }
    return client;
  }
}

/** See TopicGateway in rosTopicGateway.ts for why this `Pick` exists. */
export type ActionGateway = Pick<RosActionGateway, "sendGoal" | "cancelGoal">;
