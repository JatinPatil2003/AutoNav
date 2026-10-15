import { baseApi } from "../../app/baseApi";
import { UserRole } from "../auth/auth.types";
import {
  AdminRobot,
  AdminUser,
  CreateRobotRequest,
  CreateRobotResult,
  RobotAccessRequest,
  RobotAssignment,
} from "./admin.types";

export const adminApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    listPendingRequests: builder.query<RobotAccessRequest[], void>({
      query: () => "/admin/robot-access-requests?status=pending",
      transformResponse: (response: { requests: RobotAccessRequest[] }) => response.requests,
      providesTags: ["AdminRobotAccessRequests"],
    }),
    approveRequest: builder.mutation<RobotAccessRequest, { id: string; robotId?: string }>({
      query: ({ id, robotId }) => ({
        url: `/admin/robot-access-requests/${id}/approve`,
        method: "POST",
        body: { robotId },
      }),
      transformResponse: (response: { request: RobotAccessRequest }) => response.request,
      invalidatesTags: ["AdminRobotAccessRequests", "AdminRobots"],
    }),
    rejectRequest: builder.mutation<RobotAccessRequest, { id: string; reason?: string }>({
      query: ({ id, reason }) => ({
        url: `/admin/robot-access-requests/${id}/reject`,
        method: "POST",
        body: { reason },
      }),
      transformResponse: (response: { request: RobotAccessRequest }) => response.request,
      invalidatesTags: ["AdminRobotAccessRequests"],
    }),
    createRobot: builder.mutation<CreateRobotResult, CreateRobotRequest>({
      query: (body) => ({ url: "/admin/robots", method: "POST", body }),
      invalidatesTags: ["AdminRobots"],
    }),
    listAllRobots: builder.query<AdminRobot[], void>({
      query: () => "/admin/robots",
      transformResponse: (response: { robots: AdminRobot[] }) => response.robots,
      providesTags: ["AdminRobots"],
    }),
    listUsers: builder.query<AdminUser[], void>({
      query: () => "/admin/users",
      transformResponse: (response: { users: AdminUser[] }) => response.users,
      providesTags: ["AdminUsers"],
    }),
    updateUserRole: builder.mutation<AdminUser, { id: string; role: UserRole }>({
      query: ({ id, role }) => ({ url: `/admin/users/${id}/role`, method: "PATCH", body: { role } }),
      transformResponse: (response: { user: AdminUser }) => response.user,
      invalidatesTags: ["AdminUsers"],
    }),
    grantRobotAssignment: builder.mutation<
      RobotAssignment,
      { userId: string; robotId: string; hmiAccess?: boolean; platformAccess?: boolean }
    >({
      query: (body) => ({ url: "/admin/robot-assignments", method: "POST", body }),
      transformResponse: (response: { assignment: RobotAssignment }) => response.assignment,
      invalidatesTags: ["AdminUsers", "AdminRobots", "UserAssignments", "RobotAssignments"],
    }),
    updateRobotAssignmentAccess: builder.mutation<
      RobotAssignment,
      { id: string; hmiAccess?: boolean; platformAccess?: boolean }
    >({
      query: ({ id, ...body }) => ({
        url: `/admin/robot-assignments/${id}/access`,
        method: "PATCH",
        body,
      }),
      transformResponse: (response: { assignment: RobotAssignment }) => response.assignment,
      invalidatesTags: ["AdminUsers", "AdminRobots", "UserAssignments", "RobotAssignments"],
    }),
    revokeRobotAssignment: builder.mutation<RobotAssignment, { id: string; reason?: string }>({
      query: ({ id, reason }) => ({
        url: `/admin/robot-assignments/${id}/revoke`,
        method: "POST",
        body: { reason },
      }),
      transformResponse: (response: { assignment: RobotAssignment }) => response.assignment,
      invalidatesTags: ["AdminUsers", "AdminRobots", "UserAssignments", "RobotAssignments"],
    }),
    listAssignmentsForUser: builder.query<RobotAssignment[], string>({
      query: (userId) => `/admin/users/${userId}/assignments`,
      transformResponse: (response: { assignments: RobotAssignment[] }) => response.assignments,
      providesTags: ["UserAssignments"],
    }),
    listAssignmentsForRobot: builder.query<RobotAssignment[], string>({
      query: (robotId) => `/admin/robots/${robotId}/assignments`,
      transformResponse: (response: { assignments: RobotAssignment[] }) => response.assignments,
      providesTags: ["RobotAssignments"],
    }),
    deleteRobot: builder.mutation<void, string>({
      query: (id) => ({ url: `/admin/robots/${id}`, method: "DELETE" }),
      invalidatesTags: ["AdminRobots", "AdminUsers", "RobotOptions"],
    }),
    deleteUser: builder.mutation<void, string>({
      query: (id) => ({ url: `/admin/users/${id}`, method: "DELETE" }),
      invalidatesTags: ["AdminUsers", "AdminRobots"],
    }),
  }),
});

export const {
  useListPendingRequestsQuery,
  useApproveRequestMutation,
  useRejectRequestMutation,
  useCreateRobotMutation,
  useListAllRobotsQuery,
  useListUsersQuery,
  useUpdateUserRoleMutation,
  useGrantRobotAssignmentMutation,
  useUpdateRobotAssignmentAccessMutation,
  useRevokeRobotAssignmentMutation,
  useListAssignmentsForUserQuery,
  useListAssignmentsForRobotQuery,
  useDeleteRobotMutation,
  useDeleteUserMutation,
} = adminApi;
