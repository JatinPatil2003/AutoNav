import * as rclnodejs from "rclnodejs";

export class FakeRosActionGateway {
  lastGoal: unknown;
  cancelledHandles: rclnodejs.ClientGoalHandle<any>[] = [];
  private resultResolvers: ((value: unknown) => void)[] = [];

  async sendGoal<TGoal, TFeedback, TResult>(
    _type: string,
    _actionName: string,
    goal: TGoal,
    _onFeedback?: (feedback: TFeedback) => void,
  ): Promise<{ handle: rclnodejs.ClientGoalHandle<any>; result: Promise<TResult> }> {
    this.lastGoal = goal;
    const handle = { status: 4 } as unknown as rclnodejs.ClientGoalHandle<any>; // STATUS_SUCCEEDED
    const result = new Promise<TResult>((resolve) => {
      this.resultResolvers.push(resolve as (value: unknown) => void);
    });
    return { handle, result };
  }

  async cancelGoal(handle: rclnodejs.ClientGoalHandle<any>): Promise<void> {
    this.cancelledHandles.push(handle);
  }

  /** Test helper: resolves the most recently returned sendGoal() result promise. */
  resolveResult(value: unknown = undefined): void {
    this.resultResolvers.pop()?.(value);
  }
}
