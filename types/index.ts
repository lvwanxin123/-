export type Prompt={id:string;title:string;content:string;category:string;likes:number;is_premium:boolean;created_at:string};
export type Subscription={id:string;user_id:string;status:string;pending_order_id:string|null;current_period_end:string|null;created_at:string};
export type UserFavorite={id:string;user_id:string;prompt_id:string;created_at:string};
